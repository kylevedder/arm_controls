//! Rust public follower API. The existing arm_controls native node exclusively
//! owns motor I/O, gravity compensation and protection. This client owns its
//! process/session and acknowledged lifecycle, never a motor control loop.
mod protocol;
mod transport;
use serde::{Deserialize, Serialize};
use std::{
    collections::HashSet,
    fs::File,
    io::{BufRead, BufReader, Write},
    os::{
        fd::{AsRawFd, FromRawFd},
        unix::process::CommandExt,
    },
    path::PathBuf,
    process::{Child, Command, Stdio},
    sync::{Arc, Condvar, Mutex, mpsc},
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};
pub type Error = Box<dyn std::error::Error + Send + Sync>;
pub type Result<T> = std::result::Result<T, Error>;
fn err(s: impl Into<String>) -> Error {
    std::io::Error::other(s.into()).into()
}
/// The CPython-compatible host monotonic epoch: Linux CLOCK_MONOTONIC,
/// macOS mach_absolute_time (also used by the VR native runtime).
#[allow(deprecated)]
pub fn monotonic_ns() -> u64 {
    #[cfg(target_os = "macos")]
    {
        let mut info = libc::mach_timebase_info_data_t { numer: 0, denom: 0 };
        assert_eq!(unsafe { libc::mach_timebase_info(&mut info) }, 0);
        assert_ne!(info.denom, 0);
        return (unsafe { libc::mach_absolute_time() } as u128 * info.numer as u128
            / info.denom as u128) as u64;
    }
    #[cfg(not(target_os = "macos"))]
    {
        let mut t = libc::timespec {
            tv_sec: 0,
            tv_nsec: 0,
        };
        assert_eq!(
            unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut t) },
            0
        );
        t.tv_sec as u64 * 1_000_000_000 + t.tv_nsec as u64
    }
}
#[derive(Clone, Debug)]
pub struct SocketCanConnection {
    pub interface: String,
}
#[derive(Clone, Debug)]
pub struct ArmConfig {
    pub name: String,
    pub model: String,
    pub connection: SocketCanConnection,
    pub executable: PathBuf,
    /// Exact attested shared library; None permits system discovery for non-VR clients/tests.
    pub zmq_library: Option<PathBuf>,
    /// None preserves ARM_CONTROLS_LOG_DIR, then ~/arm_controls-data/logs.
    pub log_directory: Option<PathBuf>,
    pub model_root: PathBuf,
    pub instance_config: PathBuf,
    pub urdf: PathBuf,
    pub torq_rescale: [f64; 6],
    pub no_recovery_motion: bool,
    pub follower_gravity_compensation: bool,
    pub connect_timeout: Duration,
}
impl ArmConfig {
    pub fn validate(&self) -> Result<()> {
        if self.model != "Yam"
            || self.name.is_empty()
            || !self
                .name
                .bytes()
                .all(|c| c.is_ascii_alphanumeric() || b"_-".contains(&c))
            || self.connection.interface.is_empty()
            || self.connection.interface.contains('/')
            || !self.no_recovery_motion
            || !self.follower_gravity_compensation
            || self.torq_rescale.iter().any(|f| !f.is_finite() || *f < 0.)
            || self.connect_timeout.is_zero()
        {
            return Err(err("invalid strict YAM follower configuration"));
        }
        for p in [
            &self.executable,
            &self.instance_config,
            &self.urdf,
            &self.model_root.join("arms/Yam/Yam.json"),
            &self.model_root.join("effectors/E_Yam/E_Yam.json"),
            &self.model_root.join("effectors/E_Yam/E_Yam_01.json"),
        ] {
            if !p.is_file() {
                return Err(err(format!("missing arm_controls asset {}", p.display())));
            }
        }
        Ok(())
    }
    fn args(&self, topic: &str) -> Vec<String> {
        let mut a = vec![
            "--role".into(),
            "follower".into(),
            "--device_type".into(),
            "arms".into(),
            "--device_model".into(),
            "Yam".into(),
            "--device_id".into(),
            "01".into(),
            "--logical_name".into(),
            self.name.clone(),
            "--control_frequency".into(),
            "200".into(),
            "--info_level".into(),
            "0".into(),
            "--topic_type".into(),
            "ZMQ".into(),
            "--algo_type".into(),
            "Pinocchio".into(),
            "--force_feedback".into(),
            "-1".into(),
            "--dont_go_to_home_pos".into(),
            "--no_recovery_motion".into(),
            "--follower_gravity_compensation".into(),
            "true".into(),
            "--control_port".into(),
            self.connection.interface.clone(),
            "--torq_rescale".into(),
            self.torq_rescale.map(|x| x.to_string()).join(","),
            "--effector_model".into(),
            "E_Yam".into(),
        ];
        for (flag, suffix) in [
            ("--topic_state", "state"),
            ("--topic_live_command", "live"),
            ("--topic_direct_command", "direct"),
            ("--topic_lifecycle", "lifecycle"),
            ("--topic_status", "status"),
        ] {
            a.extend([flag.into(), format!("{topic}.{suffix}")]);
        }
        for (flag, p) in [
            (
                "--arm_model_config",
                self.model_root.join("arms/Yam/Yam.json"),
            ),
            ("--arm_instance_config", self.instance_config.clone()),
            (
                "--effector_model_config",
                self.model_root.join("effectors/E_Yam/E_Yam.json"),
            ),
            (
                "--effector_instance_config",
                self.model_root.join("effectors/E_Yam/E_Yam_01.json"),
            ),
            ("--urdf_path", self.urdf.clone()),
        ] {
            a.extend([flag.into(), p.to_string_lossy().into_owned()]);
        }
        a
    }
}
#[derive(Clone, Debug)]
pub struct PositionCommand {
    pub position_rad: [f64; 6],
    pub effector: f64,
}
impl PositionCommand {
    pub fn new(position_rad: [f64; 6], effector: f64) -> Result<Self> {
        let c = Self {
            position_rad,
            effector,
        };
        c.validate()?;
        Ok(c)
    }
    fn validate(&self) -> Result<()> {
        if self
            .position_rad
            .iter()
            .any(|x| !x.is_finite() || !(*x as f32).is_finite())
            || !self.effector.is_finite()
            || !(0. ..=1.).contains(&self.effector)
        {
            return Err(err("invalid position command"));
        }
        Ok(())
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ArmState {
    pub positions: [f64; 7],
    pub velocity: [f64; 7],
    pub effort: [f64; 7],
    pub temperature: [f64; 7],
    pub current: [f64; 7],
    pub frame_age_ms: [f64; 7],
    pub received_mono_ns: u64,
    pub received_epoch_ns: u64,
    pub mode: String,
    pub generation: u64,
    pub sequence: i32,
    pub feedback_gap_count: u64,
    pub feedback_hold_active: bool,
}
#[derive(Clone, Debug)]
pub struct ArmCapabilities {
    pub no_recovery_motion: bool,
    pub gravity_compensation: bool,
    pub direct_commands: bool,
    pub dof: usize,
    pub has_effector: bool,
}
#[derive(Default)]
struct Snapshot {
    state: Option<ArmState>,
    capabilities: Option<ArmCapabilities>,
    ready: bool,
    error: Option<String>,
    closed: bool,
}
type Shared = Arc<(Mutex<Snapshot>, Condvar)>;
enum Request {
    Position(
        PositionCommand,
        mpsc::Sender<std::result::Result<(), String>>,
    ),
    Hold(mpsc::Sender<std::result::Result<(), String>>),
    Close(mpsc::Sender<std::result::Result<(), String>>),
}
struct Worker {
    tx: mpsc::SyncSender<Request>,
    join: Option<JoinHandle<()>>,
    shared: Shared,
}
pub struct FollowerArm {
    pub config: ArmConfig,
    topic: String,
    worker: Mutex<Option<Worker>>,
}
impl FollowerArm {
    pub fn connect(&self) -> Result<()> {
        self.connect_inner(true)
    }
    fn connect_inner(&self, require_host_bus: bool) -> Result<()> {
        let mut guard = self
            .worker
            .lock()
            .map_err(|_| err("session lock poisoned"))?;
        if guard.is_some() {
            return Err(err("follower already connected"));
        }
        self.config.validate()?;
        if require_host_bus && !cfg!(target_os = "linux") {
            return Err(err("native hardware only supported on Linux"));
        }
        let bus = PathBuf::from("/sys/class/net").join(&self.config.connection.interface);
        if require_host_bus && !bus.exists() {
            return Err(err("SocketCAN interface is unavailable"));
        }
        if let Ok(s) = std::fs::read_to_string(bus.join("operstate")) {
            if require_host_bus && !matches!(s.trim(), "up" | "unknown") {
                return Err(err("SocketCAN interface is not up"));
            }
        }
        let shared = Arc::new((Mutex::new(Snapshot::default()), Condvar::new()));
        let (tx, rx) = mpsc::sync_channel(8);
        let config = self.config.clone();
        let topic = self.topic.clone();
        let local = shared.clone();
        let join = thread::spawn(move || {
            let result = run(config, topic, rx, &local);
            let (s, c) = (&local.0, &local.1);
            let mut s = s.lock().unwrap();
            if let Err(e) = result {
                s.error = Some(e.to_string());
            }
            s.closed = true;
            c.notify_all();
        });
        let mut w = Worker {
            tx,
            join: Some(join),
            shared,
        };
        let deadline = Instant::now() + self.config.connect_timeout;
        let result = (|| -> Result<()> {
            let mut s = w.shared.0.lock().unwrap();
            loop {
                if let Some(e) = &s.error {
                    return Err(err(e.clone()));
                }
                if s.closed {
                    return Err(err("native worker exited before ready"));
                }
                if s.ready && s.capabilities.is_some() && s.state.is_some() {
                    return Ok(());
                }
                let wait = deadline
                    .checked_duration_since(Instant::now())
                    .ok_or_else(|| err("native connect handshake/ready/state timeout"))?;
                s = w.shared.1.wait_timeout(s, wait).unwrap().0;
            }
        })();
        if result.is_err() {
            let (reply, _) = mpsc::channel();
            let _ = w.tx.send(Request::Close(reply));
            if let Some(j) = w.join.take() {
                let _ = j.join();
            }
        } else {
            *guard = Some(w);
        }
        result
    }
    pub fn capabilities(&self) -> Result<ArmCapabilities> {
        let g = self.worker.lock().unwrap();
        let w = g.as_ref().ok_or_else(|| err("not connected"))?;
        let s = w.shared.0.lock().unwrap();
        if let Some(e) = &s.error {
            return Err(err(e.clone()));
        }
        s.capabilities
            .clone()
            .ok_or_else(|| err("missing handshake"))
    }
    pub fn latest_state(&self) -> Result<Option<ArmState>> {
        let g = self.worker.lock().unwrap();
        let w = g.as_ref().ok_or_else(|| err("not connected"))?;
        let s = w.shared.0.lock().unwrap();
        if let Some(e) = &s.error {
            return Err(err(e.clone()));
        }
        if s.closed {
            return Err(err("native process closed"));
        }
        Ok(s.state.clone())
    }
    pub fn read_state(&self, timeout: Duration) -> Result<ArmState> {
        let shared = {
            let g = self.worker.lock().unwrap();
            g.as_ref()
                .ok_or_else(|| err("not connected"))?
                .shared
                .clone()
        };
        let deadline = Instant::now() + timeout;
        let mut s = shared.0.lock().unwrap();
        let generation = s.state.as_ref().map(|s| s.generation);
        loop {
            if let Some(e) = &s.error {
                return Err(err(e.clone()));
            }
            if s.closed {
                return Err(err("native process closed"));
            }
            if let Some(state) = &s.state {
                if Some(state.generation) != generation {
                    return Ok(state.clone());
                }
            }
            let wait = deadline
                .checked_duration_since(Instant::now())
                .ok_or_else(|| err("state timeout"))?;
            s = shared.1.wait_timeout(s, wait).unwrap().0;
        }
    }
    fn request(
        &self,
        make: impl FnOnce(mpsc::Sender<std::result::Result<(), String>>) -> Request,
    ) -> Result<()> {
        let g = self.worker.lock().unwrap();
        let w = g.as_ref().ok_or_else(|| err("not connected"))?;
        let (tx, rx) = mpsc::channel();
        w.tx.try_send(make(tx))
            .map_err(|e| err(format!("native request unavailable: {e}")))?;
        rx.recv_timeout(Duration::from_secs(4))
            .map_err(|_| err("native request timeout"))?
            .map_err(err)
    }
    pub fn command(&self, c: PositionCommand) -> Result<()> {
        c.validate()?;
        self.request(|reply| Request::Position(c, reply))
    }
    pub fn hold(&self) -> Result<()> {
        self.request(Request::Hold)
    }
    pub fn close(&self) -> Result<()> {
        let mut g = self.worker.lock().unwrap();
        let Some(mut w) = g.take() else {
            return Ok(());
        };
        let (tx, rx) = mpsc::channel();
        let sent = w.tx.try_send(Request::Close(tx)).is_ok();
        let response = if sent {
            rx.recv_timeout(Duration::from_secs(11))
                .map_err(|_| "native shutdown worker timeout".into())
                .and_then(|r| r)
        } else {
            Err("native worker already stopped".into())
        };
        if let Some(j) = w.join.take() {
            if j.join().is_err() {
                return Err(err("native worker panicked"));
            }
        }
        let s = w.shared.0.lock().unwrap();
        if let Some(e) = &s.error {
            return Err(err(e.clone()));
        }
        response.map_err(err)
    }
}
impl Drop for FollowerArm {
    fn drop(&mut self) {
        let _ = self.close();
    }
}
pub struct ArmSession {
    pub session_id: String,
    followers: Vec<Arc<FollowerArm>>,
    names: HashSet<String>,
}
impl Default for ArmSession {
    fn default() -> Self {
        Self::new()
    }
}
impl ArmSession {
    pub fn new() -> Self {
        Self {
            session_id: uuid::Uuid::new_v4().simple().to_string(),
            followers: Vec::new(),
            names: HashSet::new(),
        }
    }
    pub fn add_follower(&mut self, config: ArmConfig) -> Result<Arc<FollowerArm>> {
        if !self.names.insert(config.name.clone()) {
            return Err(err("duplicate logical arm name"));
        }
        let arm = Arc::new(FollowerArm {
            topic: format!("arm_controls.{}.{}", self.session_id, config.name),
            config,
            worker: Mutex::new(None),
        });
        self.followers.push(arm.clone());
        Ok(arm)
    }
    pub fn connect(&self) -> Result<()> {
        self.connect_followers(|arm| arm.connect())
    }
    fn connect_followers(&self, connect: impl Fn(&FollowerArm) -> Result<()>) -> Result<()> {
        for arm in &self.followers {
            if let Err(e) = connect(arm) {
                let _ = self.close();
                return Err(e);
            }
        }
        Ok(())
    }
    pub fn close(&self) -> Result<()> {
        let mut errors = Vec::new();
        for arm in self.followers.iter().rev() {
            if let Err(e) = arm.close() {
                errors.push(e.to_string());
            }
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(err(errors.join("; ")))
        }
    }
}
impl Drop for ArmSession {
    fn drop(&mut self) {
        let _ = self.close();
    }
}
struct OwnedChild {
    child: Child,
    _liveness: File,
}
impl Drop for OwnedChild {
    fn drop(&mut self) {
        if self.child.try_wait().ok().flatten().is_none() {
            unsafe {
                libc::kill(self.child.id() as i32, libc::SIGTERM);
            }
            let deadline = Instant::now() + Duration::from_secs(5);
            while Instant::now() < deadline {
                if self.child.try_wait().ok().flatten().is_some() {
                    return;
                }
                thread::sleep(Duration::from_millis(10));
            }
            let _ = self.child.kill();
            let _ = self.child.wait();
        }
    }
}
struct Pending {
    code: i16,
    id: i32,
    reply: mpsc::Sender<std::result::Result<(), String>>,
    deadline: Instant,
    retry: Instant,
}
fn native_log(config: &ArmConfig) -> Result<Arc<Mutex<File>>> {
    use std::os::unix::fs::OpenOptionsExt;
    let directory = if let Some(path) = &config.log_directory {
        path.clone()
    } else if let Some(path) = std::env::var_os("ARM_CONTROLS_LOG_DIR") {
        PathBuf::from(path)
    } else {
        PathBuf::from(
            std::env::var_os("HOME").ok_or_else(|| err("HOME or ARM_CONTROLS_LOG_DIR required"))?,
        )
        .join("arm_controls-data/logs")
    };
    std::fs::create_dir_all(&directory)?;
    let stem = format!(
        "arm_controls_node__follower__{}__{}",
        config.name, config.model
    );
    let path = directory.join(format!("{stem}.log"));
    if path.symlink_metadata().is_ok() {
        // Preserve previous diagnostic output, including same-tick restarts.
        std::fs::rename(
            &path,
            directory.join(format!("{stem}__{}.log", uuid::Uuid::new_v4().simple())),
        )?;
    }
    let file = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .mode(0o600)
        .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
        .open(path)?;
    Ok(Arc::new(Mutex::new(file)))
}
fn run(
    config: ArmConfig,
    topic: String,
    rx: mpsc::Receiver<Request>,
    shared: &Shared,
) -> Result<()> {
    let context = transport::Context::new(config.zmq_library.as_deref())?;
    let state = transport::Socket::new(&context, &format!("{topic}.state"), false)?;
    let status = transport::Socket::new(&context, &format!("{topic}.status"), false)?;
    let direct = transport::Socket::new(&context, &format!("{topic}.direct"), true)?;
    let lifecycle = transport::Socket::new(&context, &format!("{topic}.lifecycle"), true)?;
    let mut pipes = [0; 2];
    if unsafe { libc::pipe(pipes.as_mut_ptr()) } != 0 {
        return Err(std::io::Error::last_os_error().into());
    }
    let read = unsafe { File::from_raw_fd(pipes[0]) };
    let write = unsafe { File::from_raw_fd(pipes[1]) };
    unsafe {
        libc::fcntl(write.as_raw_fd(), libc::F_SETFD, libc::FD_CLOEXEC);
    }
    let tee = native_log(&config)?;
    let mut command = Command::new(&config.executable);
    command
        .args(config.args(&topic))
        .args(["--parent_liveness_fd", &read.as_raw_fd().to_string()])
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    unsafe {
        command.pre_exec(|| {
            if libc::setsid() < 0 {
                return Err(std::io::Error::last_os_error());
            }
            Ok(())
        });
    }
    let child = command.spawn()?;
    drop(read);
    let mut child = OwnedChild {
        child,
        _liveness: write,
    };
    let (logtx, logrx) = mpsc::channel();
    for pipe in [
        child
            .child
            .stdout
            .take()
            .map(|p| Box::new(p) as Box<dyn std::io::Read + Send>),
        child
            .child
            .stderr
            .take()
            .map(|p| Box::new(p) as Box<dyn std::io::Read + Send>),
    ] {
        if let Some(pipe) = pipe {
            let tx = logtx.clone();
            let tee = tee.clone();
            thread::spawn(move || {
                for line in BufReader::new(pipe)
                    .lines()
                    .map_while(std::result::Result::ok)
                {
                    eprintln!("{line}");
                    if let Ok(mut file) = tee.lock() {
                        let _ = writeln!(file, "{line}");
                        let _ = file.flush();
                    }
                    if let Some(i) = line.find("HARDWARE FAULT:") {
                        let _ = tx.send(line[i..].to_owned());
                    }
                }
            });
        }
    }
    let mut pending: Option<Pending> = None;
    let mut request_id = 0i32;
    let mut heartbeat = Instant::now();
    let mut generation = 0;
    let (mut gaps, mut holding) = (0u64, false);
    let (mut handshake_seen, mut follower_gravity_seen) = (false, false);
    let mut closing = false;
    let mut terminal: Option<String> = None;
    let mut mode = "hold".to_string();
    loop {
        if let Ok(fault) = logrx.try_recv() {
            terminal = Some(fault);
        }
        let mut acknowledgements = Vec::new();
        for _ in 0..256 {
            let Some(b) = status.recv()? else {
                break;
            };
            let (code, _floats, ints) = protocol::status(&b)?;
            match code {
                30 => {
                    if ints.len() < 3
                        || ints[0..2] != [1, 1]
                        || ints[2] & 32 == 0
                        || ints[2] & 1 == 0
                    {
                        return Err(err("incompatible native strict/direct handshake"));
                    }
                    handshake_seen = true;
                }
                33 => {
                    // CAP_GRAVITY_COMP advertises the leader's gravity mode.
                    // Followers report applied device-level feed-forward in
                    // SERVO_PARAM slot 3 (on every arm joint's report).
                    if ints.len() < 4 || !(0..=6).contains(&ints[0]) || ints[3] != 1 {
                        return Err(err("native follower gravity compensation not confirmed"));
                    }
                    if ints[0] < 6 {
                        follower_gravity_seen = true;
                    }
                }
                1 => {
                    if terminal.is_none() {
                        shared.0.lock().unwrap().ready = true;
                    }
                }
                20 | 21 | 22 => {
                    terminal = Some(format!("native terminal fault status {code}: {ints:?}"))
                }
                32 => {
                    let modes = [
                        "disconnected",
                        "connecting",
                        "hold",
                        "direct",
                        "gravity_compensation",
                        "bilateral",
                        "move_to_ready",
                        "recovery",
                        "closed",
                    ];
                    let index = *ints.first().ok_or_else(|| err("missing native mode"))?;
                    mode = modes
                        .get(index as usize)
                        .ok_or_else(|| err("invalid native mode"))?
                        .to_string();
                    if index == 7 {
                        terminal = Some("native terminal recovery mode".into());
                    }
                }
                34 => {
                    if ints.len() != 2
                        || ints[0] < 0
                        || (ints[0] as u64) < gaps
                        || !matches!(ints[1], 0 | 1)
                    {
                        return Err(err("invalid feedback-gap status"));
                    }
                    gaps = ints[0] as u64;
                    holding = ints[1] != 0;
                }
                31 => {
                    if ints.len() < 2 {
                        return Err(err("invalid lifecycle ACK"));
                    }
                    acknowledgements.push(ints);
                }
                _ => (),
            }
        }
        if handshake_seen && follower_gravity_seen {
            shared.0.lock().unwrap().capabilities = Some(ArmCapabilities {
                no_recovery_motion: true,
                direct_commands: true,
                gravity_compensation: true,
                dof: 6,
                has_effector: true,
            });
        }
        let mut latest = None;
        for _ in 0..256 {
            match state.recv()? {
                Some(b) => latest = Some(b),
                None => break,
            }
        }
        if let Some(b) = latest {
            generation += 1;
            let mut sample = protocol::state(&b, monotonic_ns(), generation, gaps, holding)?;
            sample.mode = mode.clone();
            shared.0.lock().unwrap().state = Some(sample);
        }
        if let Some(e) = terminal.take() {
            if let Some(p) = pending.take() {
                let _ = p.reply.send(Err(e.clone()));
            }
            shared.0.lock().unwrap().error = Some(e.clone());
            shared.1.notify_all();
            return Err(err(e));
        }
        for ints in acknowledgements {
            if pending.as_ref().is_some_and(|p| p.id == ints[0]) {
                let p = pending.take().unwrap();
                if ints[1] != 0 {
                    let _ = p.reply.send(Err(format!(
                        "native command {} rejected {}",
                        p.code, ints[1]
                    )));
                    if p.code == 1 {
                        return Err(err("native shutdown rejected"));
                    }
                } else if p.code == 1 {
                    pending = Some(p);
                    closing = true;
                } else {
                    let _ = p.reply.send(Ok(()));
                }
            }
        }
        shared.1.notify_all();
        if let Some(exit) = child.child.try_wait()? {
            if closing && exit.success() {
                if let Some(p) = pending.take() {
                    let _ = p.reply.send(Ok(()));
                }
                return Ok(());
            }
            return Err(err(format!("native process exited {exit}")));
        }
        if let Some(p) = &pending {
            if Instant::now() >= p.deadline {
                let p = pending.take().unwrap();
                let _ = p.reply.send(Err("native lifecycle timeout".into()));
                return Err(err("native lifecycle timeout"));
            }
        }
        if !closing && pending.as_ref().is_some_and(|p| Instant::now() >= p.retry) {
            let p = pending.as_mut().unwrap();
            lifecycle.send(&protocol::command(p.code, p.id))?;
            p.retry = Instant::now() + Duration::from_millis(200);
        }
        if pending.is_none() {
            match rx.try_recv() {
                Ok(Request::Position(c, reply)) => {
                    let s = shared.0.lock().unwrap();
                    let r = if s.ready && s.capabilities.is_some() {
                        direct
                            .send(&protocol::target(&c, (monotonic_ns() & 0x7fffffff) as i32)?)
                            .map_err(|e| e.to_string())
                    } else {
                        Err("native not ready".into())
                    };
                    let _ = reply.send(r);
                }
                Ok(request) => {
                    let (code, reply) = match request {
                        Request::Hold(r) => (33, r),
                        Request::Close(r) => (1, r),
                        _ => unreachable!(),
                    };
                    request_id = request_id
                        .checked_add(1)
                        .ok_or_else(|| err("request identity exhausted"))?;
                    pending = Some(Pending {
                        code,
                        id: request_id,
                        reply,
                        deadline: Instant::now()
                            + Duration::from_secs(if code == 1 { 8 } else { 3 }),
                        retry: Instant::now(),
                    });
                }
                Err(mpsc::TryRecvError::Disconnected) => {
                    return Err(err("session owner disconnected"));
                }
                Err(mpsc::TryRecvError::Empty) => (),
            }
        }
        if Instant::now() >= heartbeat && !closing {
            lifecycle.send(&protocol::command(34, 0))?;
            heartbeat = Instant::now() + Duration::from_secs(1);
        }
        thread::sleep(Duration::from_millis(1));
    }
}

#[cfg(test)]
mod lifecycle_tests;
