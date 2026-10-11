//! Subprocess fixtures exercise the production transport and worker. They have
//! no CAN implementation, device descriptors or motor commands.
use super::*;
use std::{fs, io::Write, os::unix::fs::PermissionsExt, path::Path};
fn status(code: i16, values: &[i32]) -> Vec<u8> {
    let mut b = protocol::command(code, 0);
    b[4..6].copy_from_slice(&(values.len() as i16).to_ne_bytes());
    for (i, v) in values.iter().enumerate() {
        b[48 + i * 4..52 + i * 4].copy_from_slice(&v.to_ne_bytes());
    }
    b
}
#[test]
fn fake_native_process() {
    let Ok(dir) = std::env::var("ARM_CONTROLS_TEST_FIXTURE") else {
        return;
    };
    let dir = PathBuf::from(dir);
    let args: Vec<_> = fs::read_to_string(dir.join("args"))
        .unwrap()
        .lines()
        .map(str::to_owned)
        .collect();
    let arg = |key: &str| args.windows(2).find(|p| p[0] == key).unwrap()[1].clone();
    assert!(args.contains(&"--no_recovery_motion".into()));
    assert!(args.contains(&"--dont_go_to_home_pos".into()));
    assert_eq!(arg("--follower_gravity_compensation"), "true");
    fs::write(dir.join("pid"), std::process::id().to_string()).unwrap();
    let mode = fs::read_to_string(dir.join("mode")).unwrap();
    let ctx = transport::Context::new(None).unwrap();
    let states = transport::Socket::new(&ctx, &arg("--topic_state"), true).unwrap();
    let statuses = transport::Socket::new(&ctx, &arg("--topic_status"), true).unwrap();
    let lifecycle = transport::Socket::new(&ctx, &arg("--topic_lifecycle"), false).unwrap();
    let direct = transport::Socket::new(&ctx, &arg("--topic_direct_command"), false).unwrap();
    let mut log = fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(dir.join("events"))
        .unwrap();
    let until = Instant::now() + Duration::from_secs(15);
    let mut n = 0i32;
    let mut close_at = None;
    let mut bad_ack_sent = false;
    while Instant::now() < until {
        if close_at.is_some_and(|t| Instant::now() >= t) {
            return;
        }
        match n % 20 {
            0 => statuses
                .send(&status(
                    30,
                    &[1, 1, if mode == "bad-handshake" { 5 } else { 35 }],
                ))
                .unwrap(),
            4 => statuses.send(&status(1, &[])).unwrap(),
            8 => statuses.send(&status(32, &[2])).unwrap(),
            10 if mode != "missing-gravity" => statuses
                .send(&status(
                    33,
                    &[
                        0,
                        1,
                        1,
                        if mode == "gravity-off" { 0 } else { 1 },
                        30000,
                        1000,
                        1,
                        100,
                        1,
                    ],
                ))
                .unwrap(),
            12 => statuses
                .send(&status(
                    34,
                    &[
                        if mode == "feedback-gap" { 2 } else { 0 },
                        if mode == "feedback-gap" { 1 } else { 0 },
                    ],
                ))
                .unwrap(),
            _ => (),
        }
        let mut b = protocol::target(&PositionCommand::new([0.; 6], 0.5).unwrap(), n).unwrap();
        for i in 50..57 {
            b[i * 4..i * 4 + 4].copy_from_slice(&2f32.to_ne_bytes());
        }
        states.send(&b).unwrap();
        if let Some(b) = direct.recv().unwrap() {
            assert_eq!(b.len(), 252);
            writeln!(log, "position").unwrap();
        }
        while let Some(b) = lifecycle.recv().unwrap() {
            let (code, _, ids) = protocol::status(&b).unwrap();
            writeln!(log, "lifecycle:{code}").unwrap();
            log.flush().unwrap();
            if code == 34 {
                continue;
            }
            let id = ids[0];
            if mode == "exit-on-hold" && code == 33 {
                std::process::exit(3);
            }
            if mode == "wrong-ack" && code == 33 && !bad_ack_sent {
                bad_ack_sent = true;
                statuses.send(&status(31, &[id + 99, 0])).unwrap();
                continue;
            }
            if mode == "fault-on-hold" && code == 33 {
                statuses.send(&status(20, &[])).unwrap();
                statuses.send(&status(31, &[id, 0])).unwrap();
                continue;
            }
            let rejected =
                (mode == "reject-hold" && code == 33) || (mode == "reject-close" && code == 1);
            statuses
                .send(&status(31, &[id, if rejected { 1 } else { 0 }]))
                .unwrap();
            if code == 1 && !rejected {
                close_at = Some(Instant::now() + Duration::from_millis(80));
            }
        }
        n += 1;
        thread::sleep(Duration::from_millis(5));
    }
    panic!("fixture was not stopped by its owner");
}
struct Fixture {
    dir: PathBuf,
    config: ArmConfig,
}
impl Fixture {
    fn new(name: &str, mode: &str) -> Self {
        let dir = std::env::temp_dir().join(format!("arm-native-fixture-{}", uuid::Uuid::new_v4()));
        fs::create_dir(&dir).unwrap();
        for p in [
            "arms/Yam/Yam.json",
            "effectors/E_Yam/E_Yam.json",
            "effectors/E_Yam/E_Yam_01.json",
            "instance.json",
            "payload.urdf",
        ] {
            let path = dir.join(p);
            fs::create_dir_all(path.parent().unwrap()).unwrap();
            fs::write(path, b"fixture").unwrap();
        }
        fs::write(dir.join("mode"), mode).unwrap();
        let quote = |p: &Path| format!("'{}'", p.to_str().unwrap().replace('\'', "'\\''"));
        let script = format!(
            "#!/bin/sh\nprintf '%s\\n' \"$@\" > {}/args\nexport ARM_CONTROLS_TEST_FIXTURE={}\nexec {} --exact lifecycle_tests::fake_native_process --nocapture\n",
            quote(&dir),
            quote(&dir),
            quote(&std::env::current_exe().unwrap())
        );
        let executable = dir.join("fixture-node");
        fs::write(&executable, script).unwrap();
        fs::set_permissions(&executable, fs::Permissions::from_mode(0o700)).unwrap();
        let config = ArmConfig {
            name: name.into(),
            model: "Yam".into(),
            connection: SocketCanConnection {
                interface: "fixture-only-no-can".into(),
            },
            executable,
            zmq_library: None,
            log_directory: Some(dir.join("logs")),
            model_root: dir.clone(),
            instance_config: dir.join("instance.json"),
            urdf: dir.join("payload.urdf"),
            torq_rescale: [1.; 6],
            no_recovery_motion: true,
            follower_gravity_compensation: true,
            connect_timeout: Duration::from_secs(3),
        };
        Self { dir, config }
    }
    fn dead(&self) {
        if let Ok(pid) = fs::read_to_string(self.dir.join("pid")) {
            assert_eq!(
                unsafe { libc::kill(pid.parse::<i32>().unwrap(), 0) },
                -1,
                "fixture still alive"
            );
        }
    }
    fn events(&self) -> String {
        fs::read_to_string(self.dir.join("events")).unwrap_or_default()
    }
}
impl Drop for Fixture {
    fn drop(&mut self) {
        self.dead();
        let _ = fs::remove_dir_all(&self.dir);
    }
}
#[test]
fn native_transport_handshake_hold_feedback_and_cleanup() {
    for mode in [
        "normal",
        "wrong-ack",
        "feedback-gap",
        "reject-hold",
        "fault-on-hold",
        "exit-on-hold",
        "bad-handshake",
        "gravity-off",
        "missing-gravity",
    ] {
        let fixture = Fixture::new("test", mode);
        let mut session = ArmSession::new();
        let arm = session.add_follower(fixture.config.clone()).unwrap();
        let result = session.connect_followers(|a| a.connect_inner(false));
        if ["bad-handshake", "gravity-off", "missing-gravity"].contains(&mode) {
            assert!(result.is_err());
            fixture.dead();
            continue;
        }
        result.unwrap();
        assert!(arm.capabilities().unwrap().no_recovery_motion);
        assert!(arm.capabilities().unwrap().gravity_compensation);
        let s = arm.read_state(Duration::from_millis(500)).unwrap();
        assert!(s.generation > 0);
        assert_eq!(s.frame_age_ms, [2.; 7]);
        assert!(s.received_epoch_ns > 0);
        arm.command(PositionCommand::new([0.; 6], 0.25).unwrap())
            .unwrap();
        if mode == "feedback-gap" {
            let deadline = Instant::now() + Duration::from_secs(1);
            loop {
                let s = arm.latest_state().unwrap().unwrap();
                if s.feedback_gap_count == 2 {
                    assert!(s.feedback_hold_active);
                    break;
                }
                assert!(Instant::now() < deadline);
                thread::sleep(Duration::from_millis(10));
            }
        }
        let before = Instant::now();
        let held = arm.hold();
        if ["reject-hold", "fault-on-hold", "exit-on-hold"].contains(&mode) {
            assert!(held.is_err(), "{mode}");
        } else {
            held.unwrap();
        }
        if mode == "wrong-ack" {
            assert!(before.elapsed() >= Duration::from_millis(150));
        }
        let closed = session.close();
        if ["fault-on-hold", "exit-on-hold"].contains(&mode) {
            assert!(closed.is_err());
        } else {
            closed.unwrap();
        }
        fixture.dead();
        assert!(fixture.events().contains("lifecycle:33"));
    }
}
#[test]
fn two_arm_connect_rollback_and_shutdown_attempts_every_arm() {
    for failure in ["bad-handshake", "reject-close"] {
        let left = Fixture::new("left", "normal");
        let right = Fixture::new("right", failure);
        let mut session = ArmSession::new();
        session.add_follower(left.config.clone()).unwrap();
        session.add_follower(right.config.clone()).unwrap();
        let connected = session.connect_followers(|a| a.connect_inner(false));
        if failure == "bad-handshake" {
            assert!(connected.is_err());
        } else {
            connected.unwrap();
            assert!(session.close().is_err());
        }
        left.dead();
        right.dead();
        assert!(
            left.events().contains("lifecycle:1"),
            "healthy arm was not shut down"
        );
    }
}
#[test]
fn diagnostic_log_rotation_preserves_prior_session() {
    let fixture = Fixture::new("logs", "normal");
    let first = native_log(&fixture.config).unwrap();
    first
        .lock()
        .unwrap()
        .write_all(b"prior diagnostic\n")
        .unwrap();
    drop(first);
    let second = native_log(&fixture.config).unwrap();
    second
        .lock()
        .unwrap()
        .write_all(b"current diagnostic\n")
        .unwrap();
    drop(second);
    let entries: Vec<_> = fs::read_dir(fixture.config.log_directory.as_ref().unwrap())
        .unwrap()
        .map(|e| e.unwrap().path())
        .collect();
    assert_eq!(entries.len(), 2);
    assert!(
        entries
            .iter()
            .any(|p| fs::read(p).unwrap() == b"prior diagnostic\n")
    );
    assert!(
        entries
            .iter()
            .any(|p| fs::read(p).unwrap() == b"current diagnostic\n")
    );
    assert!(
        entries
            .iter()
            .all(|p| fs::metadata(p).unwrap().permissions().mode() & 0o777 == 0o600)
    );
}
