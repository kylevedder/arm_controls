use crate::{ArmState, PositionCommand, Result, err};
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
    if b.len() != 252 || i16::from_ne_bytes(b[244..246].try_into()?) != 7 {
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
    #[test]
    fn invalid_command() {
        assert!(PositionCommand::new([f64::NAN; 6], 0.).is_err());
        assert!(PositionCommand::new([0.; 6], 1.1).is_err());
    }
}
