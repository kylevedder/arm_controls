use crate::{ArmState, FeedbackSample, PositionCommand, Result, err};
pub fn command(code: i16, request: i32) -> Vec<u8> {
    let mut b = vec![0; 88];
    b[..2].copy_from_slice(&code.to_ne_bytes());
    b[4..6].copy_from_slice(&(if request == 0 { 0i16 } else { 1i16 }).to_ne_bytes());
    if request != 0 {
        b[48..52].copy_from_slice(&request.to_ne_bytes());
    }
    b
}
pub fn target(c: &PositionCommand, sequence: i32) -> Result<Vec<u8>> {
    c.validate()?;
    let mut b = vec![0; 252];
    for (i, v) in c
        .position_rad
        .iter()
        .chain(std::iter::once(&c.effector))
        .enumerate()
    {
        b[i * 4..i * 4 + 4].copy_from_slice(&(*v as f32).to_ne_bytes());
    }
    for i in 50..60 {
        b[i * 4..i * 4 + 4].copy_from_slice(&(-1f32).to_ne_bytes());
    }
    b[240..244].copy_from_slice(&sequence.to_ne_bytes());
    b[244..246].copy_from_slice(&7i16.to_ne_bytes());
    b[246..248].copy_from_slice(&1i16.to_ne_bytes());
    Ok(b)
}
pub fn status(b: &[u8]) -> Result<(i16, Vec<f32>, Vec<i32>)> {
    if b.len() != 88 {
        return Err(err("invalid native status size"));
    }
    let code = i16::from_ne_bytes(b[0..2].try_into()?);
    let nf = i16::from_ne_bytes(b[2..4].try_into()?);
    let ni = i16::from_ne_bytes(b[4..6].try_into()?);
    if !(0..=10).contains(&nf) || !(0..=10).contains(&ni) {
        return Err(err("invalid status counts"));
    }
    Ok((
        code,
        (0..nf as usize)
            .map(|i| f32::from_ne_bytes(b[8 + 4 * i..12 + 4 * i].try_into().unwrap()))
            .collect(),
        (0..ni as usize)
            .map(|i| i32::from_ne_bytes(b[48 + 4 * i..52 + 4 * i].try_into().unwrap()))
            .collect(),
    ))
}
pub fn state(b: &[u8], now: u64, generation: u64, gaps: u64, holding: bool) -> Result<ArmState> {
    if !matches!(b.len(), 252 | 496) || i16::from_ne_bytes(b[244..246].try_into()?) != 7 {
        return Err(err("invalid seven-joint state"));
    }
    let read = |start: usize| {
        std::array::from_fn(|i| {
            f32::from_ne_bytes(b[(start + i) * 4..(start + i + 1) * 4].try_into().unwrap()) as f64
        })
    };
    let s = ArmState {
        positions: read(0),
        velocity: read(10),
        effort: read(20),
        temperature: read(30),
        current: read(40),
        frame_age_ms: read(50),
        feedback_samples: feedback_samples(b, now),
        lifecycle_sequence: 0,
        lifecycle_received_mono_ns: 0,
        received_mono_ns: now,
        received_epoch_ns: std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)?
            .as_nanos() as u64,
        mode: "hold".into(),
        generation,
        sequence: i32::from_ne_bytes(b[240..244].try_into()?),
        feedback_gap_count: gaps,
        feedback_hold_active: holding,
    };
    if s.positions
        .iter()
        .chain(s.velocity.iter())
        .chain(s.effort.iter())
        .chain(s.temperature.iter())
        .chain(s.current.iter())
        .chain(s.frame_age_ms.iter())
        .any(|v| !v.is_finite())
        || s.sequence < 0
    {
        return Err(err("nonfinite/negative native feedback"));
    }
    Ok(s)
}
// Invalid/unsupported optional telemetry cannot make raw physical feedback fail.
// Legacy payloads remain readable, with no claim of coherent receive provenance.
fn feedback_samples(b: &[u8], now: u64) -> [Option<FeedbackSample>; 7] {
    if b.len() != 496 || u32::from_ne_bytes(b[252..256].try_into().unwrap()) != 1 {
        return [None; 7];
    }
    std::array::from_fn(|i| {
        let at = 256 + i * 24;
        let sample = FeedbackSample {
            position: f32::from_ne_bytes(b[at..at + 4].try_into().unwrap()) as f64,
            velocity: f32::from_ne_bytes(b[at + 4..at + 8].try_into().unwrap()) as f64,
            received_mono_ns: u64::from_ne_bytes(b[at + 8..at + 16].try_into().unwrap()),
            sample_id: u64::from_ne_bytes(b[at + 16..at + 24].try_into().unwrap()),
        };
        (sample.sample_id != 0
            && sample.received_mono_ns != 0
            && sample.received_mono_ns <= now
            && sample.position.is_finite()
            && sample.velocity.is_finite()
            && (i != 6 || (0.0..=1.0).contains(&sample.position)))
        .then_some(sample)
    })
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn exact_native_layout() {
        let b = command(33, 123);
        assert_eq!(b.len(), 88);
        assert_eq!(&b[48..52], &123i32.to_ne_bytes());
        let c = PositionCommand::new([0.; 6], 0.5).unwrap();
        let t = target(&c, 7).unwrap();
        let s = state(&t, 10, 1, 0, false).unwrap();
        assert_eq!(s.positions[6], 0.5);
        assert_eq!(s.frame_age_ms, [-1.; 7]);
        assert_eq!(s.sequence, 7);
        assert!(state(&t[..251], 0, 0, 0, false).is_err());
    }
    fn extended() -> Vec<u8> {
        let mut b = target(&PositionCommand::new([0.0; 6], 0.5).unwrap(), 7).unwrap();
        b.resize(496, 0);
        b[252..256].copy_from_slice(&1u32.to_ne_bytes());
        for i in 0..7 {
            let at = 256 + 24 * i;
            b[at..at + 4].copy_from_slice(&(if i == 6 { 0.75f32 } else { 0.2f32 }).to_ne_bytes());
            b[at + 4..at + 8].copy_from_slice(&(-0.5f32).to_ne_bytes());
            b[at + 8..at + 16].copy_from_slice(&(100u64 + i as u64).to_ne_bytes());
            b[at + 16..at + 24].copy_from_slice(&(20u64 + i as u64).to_ne_bytes());
        }
        b
    }
    #[test]
    fn coherent_samples_preserve_identity_through_duplicate_publications() {
        let b = extended();
        let a = state(&b, 200, 1, 0, false).unwrap();
        let duplicate = state(&b, 300, 2, 0, false).unwrap();
        assert_eq!(a.feedback_samples, duplicate.feedback_samples);
        assert_ne!(a.generation, duplicate.generation);
        assert_eq!(a.positions[6], 0.5); // Legacy fields remain unchanged.
        let gripper = a.feedback_samples[6].unwrap();
        assert_eq!(gripper.position, 0.75);
        assert_eq!(gripper.velocity, -0.5); // normalized opening per second
        assert_eq!(gripper.received_mono_ns, 106);
        assert_eq!(gripper.sample_id, 26);
    }
    #[test]
    fn partial_update_and_bad_telemetry_do_not_rewrite_other_samples_or_raw_state() {
        let mut b = extended();
        let initial = state(&b, 200, 1, 0, false).unwrap();
        let at = 256 + 24 * 2;
        b[at + 8..at + 16].copy_from_slice(&201u64.to_ne_bytes());
        b[at + 16..at + 24].copy_from_slice(&23u64.to_ne_bytes());
        let next = state(&b, 250, 2, 0, false).unwrap();
        assert_ne!(initial.feedback_samples[2], next.feedback_samples[2]);
        for i in [0, 1, 3, 4, 5, 6] {
            assert_eq!(initial.feedback_samples[i], next.feedback_samples[i]);
        }
        // A future timestamp invalidates only this optional diagnostic sample.
        assert!(state(&b, 200, 3, 0, false).unwrap().feedback_samples[2].is_none());
        b[252..256].copy_from_slice(&99u32.to_ne_bytes());
        let unknown = state(&b, 300, 4, 0, false).unwrap();
        assert_eq!(unknown.feedback_samples, [None; 7]);
        assert_eq!(unknown.positions, initial.positions);
        assert_eq!(
            state(&b[..252], 300, 5, 0, false).unwrap().feedback_samples,
            [None; 7]
        );
    }
    #[test]
    fn invalid_command() {
        assert!(PositionCommand::new([f64::NAN; 6], 0.).is_err());
        assert!(PositionCommand::new([0.; 6], 1.1).is_err());
    }
}
