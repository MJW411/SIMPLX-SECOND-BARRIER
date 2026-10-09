// OMX 세컨드 배리어 — 제어판
const $ = (s) => document.querySelector(s);
const $$ = (s) => document.querySelectorAll(s);
let S = null;                 // latest state from the server
let ws = null, wsDropped = false;
let seqDraft = null, seqDirty = false, selIdx = -1, edKey = "";
let clickMode = null;         // null | "calib" | "roi"
let roiClicks = [];
let evFilter = "all";
let page = "overview";
let thrShown = null;          // 입력칸에 채워 둔 거리 기준 (적용 중인 기준이 바뀌면 다시 채움)

const STATE_LABEL = { NORMAL: "정상", SLOW: "감속", STOP: "정지", CONVEYOR: "컨베이어 이상", FIRE: "화재" };
const EVENT_LABEL = {
  human_stop: "사람 근접 정지", human_slow: "사람 접근 감속", vanish: "사람 사라짐", conveyor_jam: "컨베이어 이상",
  fire: "화재", fire_end: "화재 종료", fault: "고장", estop: "비상정지", restart: "재가동",
  vanish_clear: "사라짐 확인", maintenance: "정비 모드", system: "시스템", cycle: "반복 작업", cycle_fail: "반복 중단",
  pick_fail: "집기 실패", calib: "보정", settings: "설정 변경",
  cycle_done: "작업 완료", cycle_pause: "작업 일시 정지", cycle_resume: "작업 재개", conveyor_clear: "컨베이어 이상 해제", door_manual: "출입문 수동 열기", teach_mode: "티칭 모드", robot_temp: "모터 온도", robot_angle: "관절 각도 이상", cycle_start: "작업 시작", cycle_stop: "작업 정지", clip_save: "영상 저장", robot: "로봇", robot_fault: "로봇 고장",
};
const STEP_LABEL = { move: "관절 이동", linear: "직선 이동", home: "홈 이동", grip: "그리퍼", wait_sensor2: "센서 대기 (D11)", conveyor: "컨베이어", pause: "시간 대기" };
const JOINTS = ["베이스", "어깨", "팔꿈치", "손목 기울기", "손목 회전", "그리퍼"];

// ---------------------------------------------------------------- helpers
async function api(path, body, method = "POST") {
  const opt = { method, headers: {} };
  if (body instanceof FormData) opt.body = body;
  else if (body !== undefined) { opt.headers["Content-Type"] = "application/json"; opt.body = JSON.stringify(body); }
  const r = await fetch(path, opt);
  if (r.status === 401) { showLogin(); throw new Error("로그인 필요"); }
  const j = await r.json().catch(() => ({}));
  if (!r.ok) { toast(j.detail || `오류 ${r.status}`, "error"); throw new Error(j.detail); }
  return j;
}
function toast(msg, level = "info", ms = 5000) {
  const d = document.createElement("div");
  d.className = `toast ${level}`;
  d.textContent = msg;
  $("#toasts").appendChild(d);
  setTimeout(() => d.remove(), ms);
}
const esc = (s) => String(s ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
const fx = (v, d = 3) => (v === null || v === undefined || Number.isNaN(v) ? "—" : Number(v).toFixed(d));
const deg = (r) => `${(r * 180 / Math.PI).toFixed(1)}°`;
function on(id, fn) { $(id).addEventListener("click", async (e) => { try { await fn(e); } catch (err) { console.warn(err); } }); }
function kv(el, obj) { $(el).innerHTML = Object.entries(obj).map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join(""); }
function tag(el, text, cls = "") { const t = $(el); t.textContent = text; t.className = "tag " + cls; }
function setIfIdle(id, v) { const el = $(id); if (document.activeElement !== el) el.value = v; }

// ---------------------------------------------------------------- login / boot
function showLogin() { $("#login").classList.remove("hidden"); $("#app").classList.add("hidden"); }
$("#loginForm").addEventListener("submit", async (e) => {
  e.preventDefault();
  const r = await fetch("/api/login", { method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ user: $("#lu").value, password: $("#lp").value }) });
  if (!r.ok) { $("#loginErr").textContent = "아이디 또는 비밀번호가 맞지 않습니다"; return; }
  start();
});
async function boot() { (await fetch("/api/me")).ok ? start() : showLogin(); }
function start() {
  $("#login").classList.add("hidden");
  $("#app").classList.remove("hidden");
  $("#video").src = "/video.mjpg?" + Date.now();
  let p = "overview";
  try { p = localStorage.getItem("omxPage") || "overview"; } catch (_) {}
  go(p);
  connectWS();
  loadModels();
  loadEvents();
  loadVideos();
}

// ---------------------------------------------------------------- navigation
function go(p) {
  if (!$("#page-" + p)) p = "overview";
  page = p;
  $$(".sidebar a").forEach((a) => a.classList.toggle("active", a.dataset.page === p));
  $$(".page").forEach((s) => s.classList.toggle("hidden", s.id !== "page-" + p));
  placeCamera();
  if (p === "events") loadEvents();
  if (p === "videos") loadVideos();
  if (p === "settings") loadModels();
  try { localStorage.setItem("omxPage", p); } catch (_) {}
}
$$(".sidebar a").forEach((a) => a.addEventListener("click", () => go(a.dataset.page)));
$$("#setNav button").forEach((b) => b.addEventListener("click", () => {
  $$("#setNav button").forEach((x) => x.classList.toggle("active", x === b));
  $$(".setpane").forEach((p) => p.classList.toggle("hidden", p.id !== "set-" + b.dataset.s));
  placeCamera();
}));
function placeCamera() {
  // one MJPEG stream only: move the camera element into the visible slot
  const slots = [...$$(".cam-slot")].filter((s) => s.offsetParent !== null);
  const cam = $("#cam");
  if (slots.length) { if (cam.parentElement !== slots[0]) slots[0].appendChild(cam); }
  else if (cam.parentElement.id !== "camHome") $("#camHome").appendChild(cam);
}

// ---------------------------------------------------------------- websocket
function connectWS() {
  ws = new WebSocket((location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws");
  ws.onopen = () => {
    $("#connDot").classList.add("on"); $("#connTxt").textContent = "연결됨";
    if (wsDropped) { $("#video").src = "/video.mjpg?" + Date.now(); loadModels(); loadEvents(); wsDropped = false; }
  };
  ws.onmessage = (m) => {
    const msg = JSON.parse(m.data);
    if (msg.kind === "state") { S = msg.data; render(); }
    else if (msg.kind === "event") onEvent(msg);
    else if (msg.kind === "clip") { loadEvents(); loadVideos(); }
    else if (msg.kind === "error") $("#jogBlocked").textContent = msg.message;
  };
  ws.onclose = (e) => {
    wsDropped = true;
    $("#connDot").classList.remove("on"); $("#connTxt").textContent = "다시 연결 중…";
    $("#stateBadge").textContent = "연결 끊김"; $("#stateBadge").className = "state";
    if (e.code === 4401) { showLogin(); return; }
    setTimeout(connectWS, 1500);
  };
}
function onEvent(ev) {
  if (ev.type === "cycle_done") {                // 정한 횟수를 마침 → 팝업 (확인을 누를 때까지)
    $("#doneMsg").textContent = ev.message;
    $("#doneModal").classList.remove("hidden");
  }
  const lvl = ev.level === "alarm" ? "alarm" : ev.level === "warn" ? "warn" : "info";
  toast(`${EVENT_LABEL[ev.type] || ev.type}: ${ev.message}`, lvl, lvl === "alarm" ? 10000 : 5000);
  if (lvl === "alarm") {
    if (navigator.vibrate) navigator.vibrate([300, 100, 300]);
    if (window.Notification && Notification.permission === "granted") {
      navigator.serviceWorker?.getRegistration().then((reg) => {
        const opt = { body: ev.message, tag: "omx-" + ev.id, icon: "/static/icon-192.png" };
        reg ? reg.showNotification("OMX 안전 경보", opt) : new Notification("OMX 안전 경보", opt);
      });
    }
  }
  loadEvents();
}

// ---------------------------------------------------------------- render
function render() {
  const sf = S.safety, rb = S.robot, pc = S.perception, sy = S.sys || {}, cal = pc.calib;
  // 화면 경보: 화재 또는 사람 근접 정지 → 붉은 깜빡임 (클릭은 막지 않음, 재가동·화재 종료 시 꺼짐)
  const alarmMsg = sf.fire ? "화재 감지 — 로봇 홈 복귀, 대피하세요" : sf.human_alarm ? "사람 근접 — 로봇 정지, 확인 후 재가동하세요" : "";
  $("#alarmFx").classList.toggle("hidden", !alarmMsg);
  $("#alarmFx").classList.toggle("fire", !!sf.fire);
  if (alarmMsg) $("#alarmTxt").textContent = alarmMsg;
  // header
  const b = $("#stateBadge");
  b.textContent = STATE_LABEL[sf.state] || sf.state; b.className = "state " + sf.state;
  $("#minDist").textContent = sf.min_dist === null ? "사람 없음" : `가장 가까운 사람 ${fx(sf.min_dist, 2)} m`;
  $("#profileTxt").textContent = `${sf.profile === "figure" ? "피규어 기준" : "실제 사람 기준"} · 감속 ${sf.thresholds.slow} m · 정지 ${sf.thresholds.stop} m · ` +
    (cal.calibrated && cal.valid !== false ? "로봇 팔까지 거리" : cal.calibrated ? "카메라까지 거리 (보정 틀어짐)" : "카메라까지 거리 (보정 전)");
  $("#reasons").textContent = sf.reasons.join(" · ") || "이상 없음";
  const un = $("#unacked"); un.textContent = S.db.unacked; un.classList.toggle("hidden", !S.db.unacked);

  // overview
  const lb = $("#latchBox");
  if (sf.stop_latched) {
    lb.classList.remove("hidden");
    const auto = /비상정지|고장|사라짐/;          // 재가동 버튼이 알아서 처리하는 항목
    const hard = sf.restart_blockers.filter((x) => !auto.test(x));
    lb.classList.toggle("ready", !hard.length);
    lb.innerHTML = `<b>정지 중:</b> ${esc(sf.stop_latched)}` + (hard.length
      ? `<div>아래 조건이 풀리면 재가동할 수 있습니다.</div><ul>${hard.map((x) => `<li>${esc(x)}</li>`).join("")}</ul>`
      : `<div><b>재가동</b>을 누르면 비상정지·고장 해제와 토크 켜기까지 한 번에 합니다.</div>`);
  } else lb.classList.add("hidden");
  tag("#calBadge", !cal.calibrated ? "보정 전" : cal.valid === false ? "보정 틀어짐" : "보정 완료", cal.calibrated && cal.valid !== false ? "ok" : "warn");
  const calBad = cal.calibrated && cal.valid === false;
  $("#calWarn").classList.toggle("hidden", cal.calibrated && !calBad);
  $("#calWarn").innerHTML = calBad
    ? `카메라 위치가 바뀌어 <b>보정이 맞지 않습니다</b> (일치율 ${Math.round((cal.fit ?? 0) * 100)}%). 지금은 카메라까지 거리로 판단합니다. <b>설정 → 보정 → 자동 보정 시작</b>으로 다시 보정하세요.`
    : `보정이 없어 <b>카메라까지 거리</b>로 판단하고 있습니다. 로봇 옆의 사람을 제대로 판단하려면 <b>설정 → 보정 → 자동 보정 시작</b>을 하세요.`;
  $$("[data-teachmode]").forEach((b) => {
    b.classList.toggle("on", !!sf.teach_mode);
    b.textContent = sf.teach_mode ? "티칭 모드 끄기" : (b.closest(".teachmode-card") ? "티칭 모드 켜기" : "티칭 모드");
  });
  tag("#teachModeTag", sf.teach_mode ? "켜짐 — 사람 감지 안 함" : "꺼짐", sf.teach_mode ? "warn" : "");
  $("#btnMaint").classList.toggle("on", sf.maintenance);
  $("#btnMaint").textContent = sf.maintenance ? "정비 모드 끝내기" : "정비 모드";
  $("#btnRestart").disabled = !sf.stop_latched;
  $("#topRestart").classList.toggle("hidden", !sf.stop_latched || sf.fire);
  $("#btnFireEnd").classList.toggle("hidden", !sf.fire);
  drawGauge($("#distGauge"), sf.thresholds, sf.min_dist, sf.zone);
  if (page === "overview") drawZoneMap($("#zoneMap"), S.map, sf.thresholds);
  kv("#safetyKv", {
    "구역 판정": { CLEAR: "여유", SLOW: "감속 구역", STOP: "정지 구역" }[sf.zone] || sf.zone,
    "사라짐": `미확인 ${sf.vanish_unresolved}건 · 판정 대기 ${sf.vanish_pending}건`,
    "컨베이어": esc(sf.conveyor_alarm || "정상"), "조명 밝기": sf.brightness ?? "—",
    "무인 시험 감시": sf.guard_person_m ? `${sf.guard_person_m} m` : "꺼짐",
  });
  const mode = rb.freedrive ? "직접 교시 중" : rb.torque ? "토크 켜짐" : "토크 꺼짐";
  const real = S.robot_backend.startsWith("real");
  tag("#robotTag", real ? "실제 로봇" : "가상 로봇", real ? "info" : "warn");
  kv("#robotKv", {
    "상태": mode, "동작": esc(rb.motion || "대기"), "속도": `${Math.round(rb.speed_ratio * 100)}% (현재 배율 ×${rb.scale})`,
    "구동 허가": esc(rb.permit_reason), "고장": rb.fault ? `<span class="err">${esc(rb.fault)}</span>` : "없음",
    "TCP": `x ${fx(rb.tcp[0])} · y ${fx(rb.tcp[1])} · z ${fx(rb.tcp[2])} m`,
  });
  $("#ovTorque").classList.toggle("hidden", rb.torque || rb.freedrive);
  const temps = rb.temp.length ? Math.max(...rb.temp) : "—";
  $("#kpis").innerHTML = [
    ["인식 속도", `${pc.fps}<small>fps</small>`], ["추론 시간", `${pc.infer_ms}<small>ms</small>`],
    ["감지된 사람", `${pc.persons}<small>명</small>`], ["모터 최고 온도", `${temps}<small>°C</small>`],
    ["CPU", `${sy.cpu ?? "—"}<small>%</small>`], ["GPU", `${sy.gpu ?? "—"}<small>%</small>`],
    ["RAM", `${sy.ram_used_gb?.toFixed?.(1) ?? "—"}<small>/ ${sy.ram_total_gb?.toFixed?.(0) ?? "—"} GB</small>`], ["프로그램 메모리", `${sy.proc_rss_mb ?? "—"}<small>MB</small>`],
  ].map(([k, v]) => `<div class="kpi"><span>${k}</span><b>${v}</b></div>`).join("");

  // robot
  tag("#torqueTag", mode, rb.freedrive ? "warn" : rb.torque ? "ok" : "");
  const sp = $("#speed");
  if (document.activeElement !== sp) { sp.max = Math.round(rb.speed_cap * 100); sp.value = Math.round(rb.speed_ratio * 100); }
  $("#speedTxt").textContent = `${Math.round(rb.speed_ratio * 100)}%`;
  $("#speedCap").textContent = `${Math.round(rb.speed_cap * 100)}%`;
  kv("#motionKv", { "비상정지": rb.estop ? '<span class="err">눌림</span>' : "해제", "휴식 자세": rb.at_rest ? "예" : "아니오",
    "제어 주기": `${rb.loop_ms} ms`, "알림": esc(rb.stall_note || "—") });
  $("#jointList").innerHTML = JOINTS.map((n, i) => {
    const load = Math.round((rb.load[i] ?? 0) * 100), t = rb.temp[i] ?? 0;
    const cls = load > 70 ? "hot" : load > 40 ? "mid" : "";
    return `<div class="joint"><span class="n">${n}</span><span class="v">${fx(rb.q[i])}</span>` +
      `<div class="bar" title="부하 ${load}%"><i class="${cls}" style="width:${Math.min(100, load)}%"></i></div>` +
      `<span class="t ${t >= 55 ? "hot" : ""}">${t}°C</span></div>`;
  }).join("") + `<p class="hint">각도(rad) · 막대 = 모터 부하 · 온도</p>`;
  const tcpHtml = [["x (m)", fx(rb.tcp[0])], ["y (m)", fx(rb.tcp[1])], ["z (m)", fx(rb.tcp[2])], ["기울기", deg(rb.tcp[3])], ["그리퍼", fx(rb.q[5], 2)]]
    .map(([k, v]) => `<div><span>${k}</span><b>${v}</b></div>`).join("");
  $("#tcpBig").innerHTML = tcpHtml; $("#tcpTeach").innerHTML = tcpHtml;

  // teach
  $("#jogBlocked").textContent = rb.jog_blocked ? `조그 제한: ${rb.jog_blocked}` : "";
  const fd = $("#btnFreedrive");
  fd.classList.toggle("on", rb.freedrive);
  fd.textContent = rb.freedrive ? "직접 교시 끝내기 (이 자리에서 고정)" : "직접 교시 (프리드라이브)";
  renderPoses();
  if (seqDraft === null) { seqDraft = JSON.parse(JSON.stringify(S.teach.sequence)); seqDirty = false; edKey = ""; }
  const gp = gripPctOf(rb.q[5]);
  const Gs = S.teach.gripper, gBad = Math.abs(Gs.open - Gs.close) < 0.15;
  $("#gripNow").textContent = gp == null ? `위치 ${fx(rb.q[5], 2)} rad · 힘 ${rb.grip_force}%` : `지금 ${gp}% 열림 · 힘 ${rb.grip_force}%`;
  $("#gripWarn").classList.toggle("hidden", !gBad);
  if (gBad) $("#gripWarn").textContent = `가동 범위가 설정되지 않았습니다 (열림 끝 ${Gs.open} · 닫힘 끝 ${Gs.close} rad). ` +
    "이대로는 열기·닫기가 움직이지 않습니다. 아래 가동 범위에서 닫힘 끝과 열림 끝을 저장하세요.";
  $("#gripRangeNow").textContent = gBad ? "설정 필요" : `닫힘 ${fx(Gs.close, 2)} ~ 열림 ${fx(Gs.open, 2)} rad · 지금 ${fx(rb.q[5], 2)} rad`;
  setIfIdle("#gripCloseIn", Gs.close); setIfIdle("#gripOpenIn", Gs.open);
  renderJogBars(rb); renderGrips();
  setIfIdle("#gripForce", rb.grip_force); if (document.activeElement !== $("#gripForce")) $("#gripForceTxt").textContent = `${rb.grip_force}%`;
  setIfIdle("#gripDefForce", S.teach.gripper.force ?? 50); if (document.activeElement !== $("#gripDefForce")) $("#gripDefForceTxt").textContent = `${S.teach.gripper.force ?? 50}%`;
  setIfIdle("#gripPct", gp ?? 0);
  if (document.activeElement !== $("#gripPct")) $("#gripPctTxt").textContent = `${$("#gripPct").value}%`;
  renderSeq();
  $("#gripCheck").checked = S.cycle.grip_check;

  // program
  const c = S.cycle;
  tag("#cycleTag", c.paused ? "일시 정지" : c.running ? "실행 중" : c.done ? "작업 완료" : "대기",
      c.paused ? "warn" : c.running || c.done ? "ok" : "");
  kv("#cycleKv", { "진행 상황": esc(c.status), "완료 횟수": c.max_cycles ? `${c.cycles} / ${c.max_cycles}회` : `${c.cycles}회`,
    "경과 시간": c.running ? `${c.elapsed}초` : "—" });

  // I/O
  const d = S.devices;
  tag("#devTag", d.backend === "virtual" ? "가상 장치" : d.backend, d.healthy ? "info" : "bad");
  const map = { sensor1: d.sensor1, sensor2: d.sensor2, fire: d.fire, door: d.door_open, conveyor: d.conveyor_on };
  $$(".tile").forEach((el) => {
    const v = map[el.dataset.dev], k = el.dataset.dev;
    el.classList.toggle("on", v);
    el.querySelector("b").textContent = k === "door" ? (v ? "열림" : "닫힘") : k === "conveyor" ? (v ? "가동 중" : "멈춤") : (v ? "감지" : "없음");
  });
  const LAMP = { G: "백색 · 대기", B: "백색 · 작업 중", C: "초록 · 작업 완료", O: "주황 · 사람 감속", R: "빨강 · 사람 정지(재가동 전까지)", Y: "노란 물결 · 화재 대피 유도" };
  const lc = S.site?.led || d.led || "G";
  $("#lampTile b").innerHTML = `<i class="lamp-dot lamp-${lc}"></i>${LAMP[lc] || lc}`;
  $("#devHint").textContent = d.backend === "virtual"
    ? "가상 장치입니다. 누를 때마다 켜짐/꺼짐이 바뀝니다. 작업 중에는 도착 센서가 꺼져 있으면 컨베이어가 돌고, 켜면 멈춥니다. 화재를 켜면 출입문이 열리고 로봇이 홈으로 돌아갑니다."
    : "아두이노 실제 장치입니다. 센서 값은 실제 센서(투입 D12·도착 D11)를 따릅니다. 작업 중 투입 센서에 물품이 올라오면 컨베이어가 돌고, 도착 센서가 감지하거나 작업이 끝나면 멈춥니다. 출입문을 누르면 닫힘 시간 동안 열립니다.";
  kv("#devKv", { "통신": esc(d.health), ...(d.flame != null ? { "화염 센서 값": `${d.flame} (300 넘으면 불꽃 — 센서 보류 중이면 감지 꺼짐)` } : {}) });
  const st = S.site || {};
  const h = st.helmet;
  tag("#helmetTag", !h ? "꺼짐" : st.door_wanted ? "문 열림" : "닫힘", !h ? "" : st.door_wanted ? "ok" : "");
  kv("#helmetKv", h ? { "판정": esc(h.state), "검출": h.detections.length ? esc(h.detections.map((x) => `${x.name} ${Math.round(x.conf * 100)}%`).join(", ")) : "—",
    "닫힐 때까지": st.door_wanted ? `${fx(Math.max(h.open_left, st.manual_left), 1)}초` : "—",
    "카메라": h.camera_error ? esc(h.camera_error) : `${h.camera_fps} fps · ${esc(h.model)} ${h.infer_ms} ms` }
    : { "판정": "헬멧 게이트가 꺼져 있습니다 (모노 카메라 또는 모델 없음)" });
  setIfIdle("#doorClose", st.close_sec);
  const cv = S.conveyor;
  drawConvChart($("#convChart"), cv);
  tag("#convTag", sf.conveyor_alarm ? "이상" : cv.timing ? "가동 중" : "멈춤", sf.conveyor_alarm ? "bad" : cv.timing ? "info" : "");
  kv("#convKv", { "제한시간": `${cv.timeout}초`, "측정": cv.timing ? "컨베이어 가동, 도착 센서 기다리는 중…" : "—",
    "최근 도착까지": cv.last_travel ? `${fx(cv.last_travel, 2)}초` : "—", "이상": esc(sf.conveyor_alarm || "없음"), "주의": esc(cv.warning || "없음") });
  $("#roiInfo").textContent = S.door_roi ? `[${S.door_roi.join(", ")}]` : "지정 안 됨";

  // settings
  $$("#profSeg button").forEach((x) => x.classList.toggle("active", x.dataset.p === sf.profile));
  if (thrShown !== sf.profile && S.safety_profiles) { thrShown = sf.profile; fillThr(); }
  $("#thrNow").innerHTML = `<b>${sf.profile === "figure" ? "피규어 시연" : "실제 사람"}</b> 적용 중 — 감속 ${sf.thresholds.slow} m · ` +
    `정지 ${sf.thresholds.stop} m · 재가동 허용 ${sf.thresholds.clear} m`;
  setIfIdle("#guardM", sf.guard_person_m);
  thrPreview();
  tag("#calTag", !cal.calibrated ? "보정 전" : cal.valid === false ? "보정 틀어짐" : "보정 완료", cal.calibrated && cal.valid !== false ? "ok" : "warn");
  kv("#calKv", {
    "수동 보정 점": `${cal.samples}개`,
    "오차": cal.residual ? `중앙값 ${(cal.residual.mean * 1000).toFixed(1)} mm · 95% ${(cal.residual.max * 1000).toFixed(1)} mm (${cal.residual.method === "auto" ? "자동" : "수동"})` : "—",
    "보정 시각": cal.t || "—", "영상 일치율": cal.fit == null ? "—" : `${Math.round(cal.fit * 100)}% (${cal.valid ? "맞음" : "틀어짐"})`,
    "자동 보정": esc(cal.auto || "—"),
  });
  kv("#modelKv", { "사용 중": esc(pc.model), "상태": esc(pc.model_status), "엔진 변환": esc(pc.convert || "—"),
    "로봇 오검출 걸러냄": `${pc.robot_rejects ?? 0}건` });
  if (pc.conf !== undefined) setIfIdle("#confPct", Math.round(pc.conf * 100));
  const rc = S.recording;
  setIfIdle("#recPre", rc.pre_sec); setIfIdle("#recPost", rc.post_sec); setIfIdle("#recConv", rc.conveyor_post_sec); setIfIdle("#convTimeout", cv.timeout);
  kv("#recKv", { "순환 녹화": `조각 ${rc.segments}개 · ${rc.fps} fps`, "프레임 압축": `${rc.enc_ms} ms`, "만드는 중인 클립": `${rc.jobs}개`,
    "저장된 이벤트": `${S.db.events}건 (미확인 ${S.db.unacked}건)`, "저장 공간 한도": `${S.db.budget_gb} GB`,
    ...(S.db.prune?.warn ? { "저장 공간": `<span class="err">${esc(S.db.prune.warn)}</span>` } : {}) });
  const bar = (label, pct, txt) => `<div class="barrow"><span>${label}</span><div class="bar"><i class="${pct > 85 ? "hot" : pct > 65 ? "mid" : ""}" style="width:${Math.min(100, pct || 0)}%"></i></div><span>${txt}</span></div>`;
  $("#sysBars").innerHTML = bar("CPU", sy.cpu, `${sy.cpu ?? "—"}%`) + bar("GPU", sy.gpu, `${sy.gpu ?? "—"}%`) +
    bar("RAM", sy.ram_total_gb ? 100 * sy.ram_used_gb / sy.ram_total_gb : 0, `${sy.ram_used_gb ?? "—"} GB`);
  const nodes = Object.entries(sy.nodes || {}).map(([k, v]) => `${esc(k)} ${v.mb}MB·${v.cpu}%`).join(" · ");
  kv("#sysKv", { "이 프로그램": `${sy.proc_rss_mb} MB · CPU ${sy.proc_cpu}%`, ...(nodes ? { "노드별": nodes } : {}),
    "온도": Object.entries(sy.temp || {}).map(([k, v]) => `${k} ${v}°C`).join(" · "), "로봇 연결": esc(S.robot_backend) });
}

function renderPoses() {
  const T = S.teach, rows = Object.entries(T.poses);
  $("#homeTxt").textContent = T.home ? `홈 자세: ${T.home}` : "홈 자세 없음";
  const html = rows.length ? rows.map(([n, p]) =>
    `<div class="row-item"><span class="name">${esc(n)}</span>${T.home === n ? '<span class="pill-home">홈</span>' : ""}` +
    `<span class="meta">${p.tcp.slice(0, 3).map((v) => v.toFixed(3)).join(", ")}</span>` +
    `<button class="btn" data-go="${esc(n)}">이동</button><button class="btn ghost" data-home="${esc(n)}">홈으로 지정</button>` +
    `<button class="btn ghost" data-del="${esc(n)}">삭제</button></div>`).join("") : '<div class="seq-empty">저장된 자세가 없습니다. 조그로 팔을 옮긴 뒤 저장하세요.</div>';
  if ($("#poseList").dataset.h !== html) { $("#poseList").innerHTML = html; $("#poseList").dataset.h = html; renderEditor(); }
}
$("#poseList").addEventListener("click", async (e) => {
  const b = e.target.closest("button"); if (!b) return;
  try {
    if (b.dataset.go) await api("/api/robot/goto", { pose: b.dataset.go });
    if (b.dataset.home) { await api("/api/poses/home", { name: b.dataset.home }); toast(`"${b.dataset.home}"을(를) 홈 자세로 지정했습니다`, "ok"); }
    if (b.dataset.del && confirm(`"${b.dataset.del}" 자세를 삭제할까요?`)) await api("/api/poses/delete", { name: b.dataset.del });
  } catch (_) {}
});

// ---------------------------------------------------------------- program editor (UR PolyScope 방식)
// 자세 이동·홈 이동은 팔만, 그리퍼는 그리퍼 단계로만. 단계를 눌러 선택 → 오른쪽에서 편집.
function gripPctOf(q5) {
  const G = S?.teach.gripper; if (!G || G.open === G.close) return null;
  return Math.round(Math.min(100, Math.max(0, 100 * (q5 - G.close) / (G.open - G.close))));
}
function stepLabel(s) {
  if (s.type === "grip" && !s.preset && (s.value === "open" || s.value === "close")) return s.value === "open" ? "그리퍼 열기" : "그리퍼 닫기";
  return STEP_LABEL[s.type];
}
function stepArg(s) {
  const sp = s.speed && s.speed < 100 ? ` · 속도 ${s.speed}%` : "";
  if (s.type === "move" || s.type === "linear") return (s.pose ?? "") + sp;
  if (s.type === "home") return (S?.teach.home || "홈 자세 미지정") + sp;
  if (s.type === "grip") {
    if (s.preset) { const g = S?.teach.grips?.[s.preset]; return `${s.preset}` + (g ? ` (열림 ${gripPctOf(g.pos) ?? "—"}% · 힘 ${g.force}%)` : " (없음!)"); }
    if (s.value === "open" || s.value === "close") return `힘 ${s.force ?? S?.teach.gripper.force ?? 50}%` + (s.force == null ? " (기본)" : "");
    return `${s.value}% 열림` + (s.force != null ? ` · 힘 ${s.force}%` : "");
  }
  if (s.type === "conveyor") return s.value === "off" ? "멈춤" : "가동";
  if (s.type === "pause") return `${s.value}초`;
  return "";
}
function newStep(t) {
  const poses = Object.keys(S?.teach.poses || {});
  if ((t === "move" || t === "linear") && !poses.length) { toast("먼저 티칭에서 자세를 저장하세요", "warn"); return null; }
  return { move: { type: t, pose: poses[0], speed: 100 }, linear: { type: t, pose: poses[0], speed: 50 }, home: { type: t, speed: 100 },
    grip: { type: "grip", value: 50 }, grip_open: { type: "grip", value: "open" }, grip_close: { type: "grip", value: "close" }, pause: { type: t, value: 1 }, wait_sensor2: { type: t }, conveyor: { type: t, value: "on" } }[t];
}
function renderSeq() {
  const cur = S?.cycle.running ? S.cycle.step : -1;
  if (selIdx >= (seqDraft || []).length) selIdx = (seqDraft || []).length - 1;
  const html = (seqDraft || []).length ? seqDraft.map((s, i) =>
    `<li data-i="${i}" class="t-${s.type} ${cur === i ? "current" : ""} ${selIdx === i ? "sel" : ""}"><span class="lbl">${stepLabel(s)}</span>` +
    `<span class="arg">${esc(stepArg(s))}</span></li>`).join("")
    : '<div class="seq-empty">비어 있습니다. 아래에서 단계를 추가하거나 픽앤플레이스 템플릿을 쓰세요.</div>';
  if ($("#seqList").dataset.h !== html) { $("#seqList").innerHTML = html; $("#seqList").dataset.h = html; }
  $("#seqDirty").classList.toggle("hidden", !seqDirty);
  renderEditor();
}
function renderEditor() {
  const s = seqDraft?.[selIdx];
  const key = s ? `${selIdx}:${s.type}:${Object.keys(S?.teach.poses || {}).join(",")}:${Object.keys(S?.teach.grips || {}).join(",")}` : "none";
  ["#btnStepUp", "#btnStepDown", "#btnStepDup", "#btnStepDel", "#btnStepRun", "#btnRunFrom"].forEach((id) => { $(id).disabled = !s; });
  if (key === edKey) return;                   // 입력 중인 칸을 다시 그리지 않는다
  edKey = key;
  $("#edTitle").textContent = s ? `${selIdx + 1}단계 · ${stepLabel(s)}` : "단계 편집";
  if (!s) { $("#stepEditor").innerHTML = '<div class="seq-empty">왼쪽에서 단계를 누르세요</div>'; return; }
  const poseOpts = Object.keys(S?.teach.poses || {}).map((n) => `<option ${n === s.pose ? "selected" : ""}>${esc(n)}</option>`).join("");
  const speed = `<label class="slider">속도 <input type="range" data-f="speed" min="5" max="100" step="5" value="${s.speed ?? 100}"><b data-show="speed">${s.speed ?? 100}%</b></label>`;
  const note = '<p class="hint">팔만 움직입니다. 그리퍼는 그대로 두니, 열고 닫으려면 그리퍼 단계를 넣으세요.</p>';
  let h = "";
  if (s.type === "move" || s.type === "linear")
    h = `<label class="field"><span>목표 자세</span><select data-f="pose">${poseOpts}</select></label>${speed}` +
      (s.type === "linear" ? '<p class="hint">그리퍼 끝이 직선으로 움직입니다(집기·놓기 접근용).</p>' : "") + note;
  else if (s.type === "home") h = `<p>홈 자세: <b>${esc(S?.teach.home || "지정 안 됨 — 티칭에서 '홈으로 지정'")}</b></p>${speed}${note}`;
  else if (s.type === "grip") {
    const grips = Object.keys(S?.teach.grips || {});
    const mode = s.preset ? "preset" : s.value === "open" || s.value === "close" ? s.value : "pct";
    const defF = S?.teach.gripper.force ?? 50;
    const G = S?.teach.gripper;
    if (G && Math.abs(G.open - G.close) < 0.15) h += '<p class="err small">그리퍼 가동 범위가 설정되지 않아 열기·닫기가 움직이지 않습니다. 티칭 → 그리퍼 → 가동 범위에서 먼저 저장하세요.</p>';
    h += `<div class="seg" data-grip>${[["preset", "저장한 동작"], ["open", "열기"], ["close", "닫기"], ["pct", "열림 정도 지정"]].map(([v, l]) =>
      `<button data-gv="${v}" class="${mode === v ? "active" : ""}">${l}</button>`).join("")}</div>`;
    if (mode === "preset") {
      h += grips.length ? `<label class="field"><span>그리퍼 동작</span><select data-f="preset">${grips.map((n) =>
        `<option ${n === s.preset ? "selected" : ""}>${esc(n)}</option>`).join("")}</select></label>` +
        '<p class="hint">저장한 위치와 힘 그대로 움직입니다. 동작은 티칭 → 그리퍼에서 만들고 고칩니다.</p>'
        : '<p class="err small">저장한 그리퍼 동작이 없습니다. 티칭 → 그리퍼에서 먼저 저장하세요.</p>';
    } else {
      if (mode === "pct") h += `<label class="slider">열림 <input type="range" data-f="value" min="0" max="100" step="5" value="${s.value}"><b data-show="value">${s.value}%</b></label>`;
      h += `<label class="slider">힘 <input type="range" data-f="force" min="5" max="100" step="5" value="${s.force ?? defF}"><b data-show="force">${s.force ?? defF}%</b></label>` +
        `<p class="hint">힘을 건드리지 않으면 기본 힘(${defF}%)을 씁니다. 열기 = 가동 범위의 열림 끝, 닫기 = 닫힘 끝(물체가 있으면 그 힘으로 쥠).</p>`;
    }
  } else if (s.type === "pause") h = `<label class="field"><span>대기 시간 (초)</span><input type="number" data-f="value" min="0.1" max="600" step="0.5" value="${s.value}"></label>`;
  else if (s.type === "wait_sensor2") h = '<p class="hint">도착 센서(D11)에 물품이 올 때까지 기다립니다. 작업 중에는 센서가 비어 있으면 컨베이어가 돌고, 물품이 감지되면 멈춥니다.</p>';
  else if (s.type === "conveyor") h = `<div class="seg" data-conv>${[["on", "가동"], ["off", "멈춤"]].map(([v, l]) =>
      `<button data-cv="${v}" class="${(s.value || "on") === v ? "active" : ""}">${l}</button>`).join("")}</div>`;
  $("#stepEditor").innerHTML = h;
}
function edited() { seqDirty = true; renderSeq(); }
$("#stepEditor").addEventListener("input", (e) => {
  const f = e.target.dataset.f, s = seqDraft?.[selIdx]; if (!f || !s) return;
  s[f] = e.target.type === "range" || e.target.type === "number" ? +e.target.value : e.target.value;
  const show = $(`#stepEditor [data-show="${f}"]`); if (show) show.textContent = `${s[f]}%`;
  edited();
});
$("#stepEditor").addEventListener("click", (e) => {
  const b = e.target.closest("button"), s = seqDraft?.[selIdx]; if (!b || !s) return;
  if (b.dataset.gv) {
    if (b.dataset.gv === "preset") {
      const first = Object.keys(S?.teach.grips || {})[0];
      delete s.value; delete s.force; if (first) s.preset = first; else s.preset = "";
    } else { delete s.preset; s.value = b.dataset.gv === "pct" ? 50 : b.dataset.gv; }
    edKey = ""; edited();
  }
  if (b.dataset.cv) { s.value = b.dataset.cv; edKey = ""; edited(); }
});
$("#seqList").addEventListener("click", (e) => {
  const li = e.target.closest("li"); if (!li) return;
  selIdx = +li.dataset.i; renderSeq();
});
function moveSel(d) {
  const j = selIdx + d; if (selIdx < 0 || j < 0 || j >= seqDraft.length) return;
  [seqDraft[selIdx], seqDraft[j]] = [seqDraft[j], seqDraft[selIdx]]; selIdx = j; edKey = ""; edited();
}
on("#btnStepUp", () => moveSel(-1));
on("#btnStepDown", () => moveSel(1));
on("#btnStepDup", () => { if (selIdx < 0) return; seqDraft.splice(selIdx + 1, 0, JSON.parse(JSON.stringify(seqDraft[selIdx]))); selIdx++; edKey = ""; edited(); });
on("#btnStepDel", () => { if (selIdx < 0) return; seqDraft.splice(selIdx, 1); selIdx = Math.min(selIdx, seqDraft.length - 1); edKey = ""; edited(); });
function addStep(t) {
  const s = newStep(t); if (!s) return;
  if (!seqDraft) seqDraft = JSON.parse(JSON.stringify(S?.teach.sequence || []));
  const at = selIdx < 0 ? seqDraft.length : selIdx + 1;
  seqDraft.splice(at, 0, s); selIdx = at; edKey = ""; edited();
}
on("#btnAddStep", () => addStep($("#stepType").value));
$$("[data-quick]").forEach((b) => b.addEventListener("click", () => addStep(b.dataset.quick)));
async function saveSeq(quiet) {
  await api("/api/sequence", { steps: seqDraft }); seqDirty = false; seqDraft = null; edKey = "";
  if (!quiet) toast("프로그램을 저장했습니다", "ok");
}
async function saveIfDirty() { if (seqDirty) { await saveSeq(true); toast("변경 내용을 저장하고 실행합니다", "info"); } }
on("#btnSeqSave", () => saveSeq(false));
// 저장하면 서버가 정리한 형태(기본 속도 등)로 다시 불러온다
on("#btnSeqTemplate", async () => {
  if (seqDraft?.length && !confirm("지금 프로그램을 픽앤플레이스 템플릿으로 바꿀까요?")) return;
  await api("/api/sequence/template"); seqDraft = null; selIdx = -1; edKey = ""; toast("픽앤플레이스 프로그램을 만들었습니다", "ok");
});
on("#btnSeqClear", () => { if (!confirm("모든 단계를 지울까요? (저장해야 반영됩니다)")) return; seqDraft = []; selIdx = -1; edKey = ""; edited(); });
on("#btnStepRun", async () => { await saveIfDirty(); await api("/api/cycle/step", { index: selIdx }); });
on("#btnRunFrom", async () => { await saveIfDirty(); await api("/api/cycle/start", { start_at: selIdx, max_cycles: 1 }); });
$("#gripCheck").addEventListener("change", (e) => api("/api/cycle/grip_check", { on: e.target.checked }).catch(() => {}));

// ---------------------------------------------------------------- actions
on("#estop", () => api("/api/robot/estop"));
async function doRestart() {
  // 버튼 하나: 비상정지·고장 해제 → 사라짐 확인(물어봄) → 토크 켜기 → 안전 조건 재검사 → 재가동
  let clear_vanish = false;
  if (S.safety.vanish_unresolved || S.safety.vanish_pending) {
    if (!confirm(`사라진 사람 ${S.safety.vanish_unresolved || S.safety.vanish_pending}명이 안전한 곳에 있는 것을 확인했나요?`)) return;
    clear_vanish = true;
  }
  const r = await api("/api/safety/restart", { clear_vanish });
  const done = r.steps?.length ? ` (${r.steps.join(" → ")})` : "";
  r.ok ? toast("재가동했습니다" + done, "ok") : toast("재가동할 수 없습니다: " + r.problems.join(" / ") + done, "warn", 9000);
}
on("#btnRestart", doRestart);
on("#topRestart", doRestart);
on("#btnMaint", () => api("/api/safety/maintenance", { on: !S.safety.maintenance }));
$$("[data-teachmode]").forEach((b) => b.addEventListener("click", () => {
  const want = !S.safety.teach_mode;
  if (want && !confirm("티칭 모드를 켜면 사람이 가까이 있어도 로봇이 멈추지 않습니다.\n로봇 옆에서 티칭하는 사람이 직접 조작할 때만 켜세요. 켤까요?")) return;
  api("/api/safety/teach_mode", { on: want }).then(() => toast(want ? "티칭 모드를 켰습니다 — 사람 감지로 멈추지 않습니다" : "티칭 모드를 껐습니다 — 사람 감지 다시 적용", want ? "warn" : "ok")).catch(() => {});
}));
on("#btnFireEnd", async () => { if (confirm("화재 상황을 종료할까요?")) await api("/api/safety/fire_end"); });
const torqueOn = () => api("/api/robot/torque", { on: true });
on("#btnTorqueOn", torqueOn); on("#ovTorque", torqueOn);
on("#btnTorqueOff", async () => {
  try { await api("/api/robot/torque", { on: false }); }
  catch (e) { if (confirm("휴식 자세가 아니라 팔이 떨어질 수 있습니다. 그래도 토크를 끌까요?")) await api("/api/robot/torque", { on: false, force: true }); }
});
on("#btnPark", () => api("/api/robot/park"));
const goHome = () => api("/api/robot/home"), stopMotion = () => api("/api/robot/stop");
on("#btnHome", goHome); on("#ovHome", goHome); on("#btnStop", stopMotion); on("#ovStop", stopMotion);
on("#btnReset", async () => { const r = await api("/api/robot/reset"); toast("해제: " + (r.result || "완료"), "ok", 8000); });
$("#speed").addEventListener("change", (e) => api("/api/robot/speed", { ratio: e.target.value / 100 }).catch(() => {}));
try { const v = localStorage.getItem("omxCycleCount"); if (v !== null) $("#cycleCount").value = v; } catch (_) {}
on("#btnCycleStart", async () => {
  const n = Math.max(0, Math.floor(Number($("#cycleCount").value) || 0));
  try { localStorage.setItem("omxCycleCount", String(n)); } catch (_) {}
  await saveIfDirty(); await api("/api/cycle/start", { max_cycles: n });
});
on("#doneOk", () => $("#doneModal").classList.add("hidden"));
on("#btnCycleOnce", async () => { await saveIfDirty(); await api("/api/cycle/start", { max_cycles: 1 }); });
on("#btnCycleStop", () => api("/api/cycle/stop"));
on("#btnSavePose", async () => {
  const n = $("#poseName").value.trim(); if (!n) return toast("자세 이름을 입력하세요", "warn");
  await api("/api/poses/save", { name: n }); toast(`"${n}" 자세를 저장했습니다`, "ok"); $("#poseName").value = "";
});
on("#btnGripOpen", () => api("/api/robot/gripper", { value: "open" }));
on("#btnGripClose", () => api("/api/robot/gripper", { value: "close" }));
$("#gripPct").addEventListener("change", (e) => api("/api/robot/gripper", { percent: +e.target.value, force: +$("#gripForce").value }).catch(() => {}));
$("#gripForce").addEventListener("input", (e) => { $("#gripForceTxt").textContent = `${e.target.value}%`; });
$("#gripForce").addEventListener("change", (e) => api("/api/robot/gripper_force", { force: +e.target.value }).catch(() => {}));
$("#gripDefForce").addEventListener("input", (e) => { $("#gripDefForceTxt").textContent = `${e.target.value}%`; });
$("#gripDefForce").addEventListener("change", (e) => api("/api/grips/default_force", { force: +e.target.value })
  .then(() => toast(`열기·닫기 기본 힘 ${e.target.value}%로 저장했습니다`, "ok")).catch(() => {}));
on("#btnGripSave", async () => {
  const name = $("#gripName").value.trim(); if (!name) return toast("동작 이름을 적으세요", "warn");
  await api("/api/grips/save", { name, force: +$("#gripForce").value });
  $("#gripName").value = ""; toast(`그리퍼 동작 "${name}"을(를) 저장했습니다`, "ok");
});
$("#gripList").addEventListener("click", async (e) => {
  const b = e.target.closest("button"); if (!b) return;
  try {
    if (b.dataset.grun) await api("/api/robot/gripper", { preset: b.dataset.grun });
    if (b.dataset.gdel && confirm(`그리퍼 동작 "${b.dataset.gdel}"을(를) 삭제할까요?`)) await api("/api/grips/delete", { name: b.dataset.gdel });
  } catch (_) {}
});
function renderGrips() {
  const G = S.teach.grips || {}, rows = Object.entries(G);
  const html = rows.length ? rows.map(([n, g]) =>
    `<div class="row-item"><span class="name">${esc(n)}</span><span class="meta">열림 ${gripPctOf(g.pos) ?? "—"}% · 힘 ${g.force}%</span>` +
    `<button class="btn" data-grun="${esc(n)}">실행</button><button class="btn ghost" data-gdel="${esc(n)}">삭제</button></div>`).join("")
    : '<div class="seq-empty">저장한 동작이 없습니다. 열림 정도·힘을 맞춘 뒤 이름을 붙여 저장하세요.</div>';
  if ($("#gripList").dataset.h !== html) { $("#gripList").innerHTML = html; $("#gripList").dataset.h = html; }
}
$("#gripPct").addEventListener("input", (e) => { $("#gripPctTxt").textContent = `${e.target.value}%`; });
on("#btnGripOpenSet", async () => { await api("/api/poses/gripper", { which: "open" }); toast("지금 위치를 그리퍼 열림 끝으로 저장했습니다", "ok"); });
on("#btnGripCloseSet", async () => { await api("/api/poses/gripper", { which: "close" }); toast("지금 위치를 그리퍼 닫힘 끝으로 저장했습니다", "ok"); });
on("#btnGripRangeSave", async () => {
  await api("/api/gripper/range", { close: +$("#gripCloseIn").value, open: +$("#gripOpenIn").value });
  toast("그리퍼 가동 범위를 저장했습니다", "ok");
});
on("#btnAlignDown", () => api("/api/robot/align", { target: "down" }));
on("#btnAlignHoriz", () => api("/api/robot/align", { target: "horizontal" }));
on("#btnFreedrive", async () => {
  const want = !S.robot.freedrive;
  if (want && !confirm("팔 관절의 힘이 빠져 팔이 처질 수 있습니다. 팔을 손으로 받친 뒤 확인을 누르세요.")) return;
  await api("/api/robot/freedrive", { on: want });
});
$$(".tile[data-dev]").forEach((el) => el.addEventListener("click", () => {
  if (el.dataset.dev === "door") return api("/api/devices/door_open").catch(() => {});
  if (S?.devices?.backend !== "virtual") return toast("실제 장치입니다 — 센서를 직접 가려 시험하세요");
  api("/api/devices/virtual", { [el.dataset.dev]: !el.classList.contains("on") }).catch(() => {});
}));
on("#btnDoorOpen", () => api("/api/devices/door_open").then(() => toast("출입문을 열었습니다", "ok")).catch(() => {}));
on("#btnDoorClose", async () => { await api("/api/settings/door", { close_sec: +$("#doorClose").value }); toast("저장했습니다", "ok"); });
// 입구 카메라: 입출력 화면을 보고 있을 때만 0.5초마다 한 장씩 받는다
setInterval(() => {
  if (page !== "io" || document.hidden || !S?.site?.helmet) return;
  const img = $("#helmetImg");
  if (img.dataset.busy) return;
  img.dataset.busy = "1";
  fetch("/api/helmet/frame.jpg", { cache: "no-store" }).then((r) => r.ok ? r.blob() : null).then((b) => {
    if (b) { const u = URL.createObjectURL(b); const old = img.src; img.src = u; if (old.startsWith("blob:")) URL.revokeObjectURL(old); }
  }).catch(() => {}).finally(() => { delete img.dataset.busy; });
}, 500);
on("#btnConvClear", () => api("/api/safety/conveyor_clear"));

// settings
function thrPreview() {                         // 입력칸 값으로 거리 기준 막대 미리 보기 (저장 전에도)
  const v = (id) => parseFloat($(id).value);
  const th = { stop: v("#thrStop"), slow: v("#thrSlow"), clear: v("#thrClear") };
  if ([th.stop, th.slow, th.clear].every(Number.isFinite)) drawGauge($("#thrGauge"), th, S?.safety?.min_dist ?? null, null, true);
}
["#thrStop", "#thrSlow", "#thrClear"].forEach((id) => $(id).addEventListener("input", thrPreview));

function fillThr() {
  const p = S?.safety_profiles?.[thrShown]; if (!p) return;
  $("#thrSlow").value = p.slow; $("#thrStop").value = p.stop; $("#thrClear").value = p.clear; $("#vanGrace").value = S.vanish_grace;
}
$$("#profSeg button").forEach((b) => b.addEventListener("click", () =>
  api("/api/safety/profile", { profile: b.dataset.p }).then(() => toast(`거리 기준: ${b.textContent}`, "ok")).catch(() => {})));
on("#btnThr", async () => {
  if (+$("#thrStop").value < 0.10 && !confirm("정지 거리가 10 cm보다 짧습니다.\n인식→판단→정지에 약 0.2초가 걸려, 손이 빠르게 다가오면 멈추기 전에 닿을 수 있습니다. 그래도 저장할까요?")) return;
  const prof = thrShown;
  await api("/api/settings/thresholds", { profile: prof, slow: +$("#thrSlow").value, stop: +$("#thrStop").value,
    clear: +$("#thrClear").value, vanish_grace: +$("#vanGrace").value });
  toast(`'${prof === "figure" ? "피규어 시연" : "실제 사람"}' 기준을 저장했고 바로 적용됩니다`, "ok");
});
on("#btnGuard", async () => { await api("/api/settings/guard", { meters: +$("#guardM").value }); toast("저장했습니다", "ok"); });
on("#btnRec", async () => {
  await api("/api/settings/recording", { pre_sec: +$("#recPre").value, post_sec: +$("#recPost").value, conveyor_post_sec: +$("#recConv").value });
  await api("/api/settings/conveyor", { travel_timeout: +$("#convTimeout").value });
  toast("녹화 설정을 저장했습니다", "ok");
});
on("#btnSysRestart", async () => {
  if (!confirm("프로그램을 재시작할까요? 약 30초 뒤 제어판이 다시 연결되고, 그동안 로봇은 제자리를 유지합니다.")) return;
  await api("/api/system/restart"); toast("재시작 중… 자동으로 다시 연결됩니다", "info", 15000);
});
on("#btnSysShutdown", async () => {
  if (!confirm("프로그램을 종료할까요? 다시 켜려면 바탕화면 아이콘이나 ./run.sh를 쓰세요.")) return;
  await api("/api/system/shutdown"); toast("프로그램을 종료했습니다", "warn", 15000);
});
on("#btnLogs", async () => {
  const r = await api("/api/logs?lines=300", undefined, "GET");
  const b = $("#logBox"); b.textContent = r.lines.join("\n"); b.classList.toggle("hidden"); b.scrollTop = b.scrollHeight;
});
on("#btnPw", async () => {
  await api("/api/settings/password", { old: $("#pwOld").value, new: $("#pwNew").value });
  $("#pwOld").value = $("#pwNew").value = ""; toast("비밀번호를 바꿨습니다", "ok");
});
on("#btnLogout", async () => { await api("/api/logout"); location.reload(); });

// ---------------------------------------------------------------- models
async function loadModels() {
  const r = await api("/api/models", undefined, "GET").catch(() => null); if (!r) return;
  $("#modelSel").innerHTML = r.models.map((m) => `<option value="${esc(m.name)}" ${m.name === r.current ? "selected" : ""}>` +
    `${esc(m.name)} — ${{ models: "~/models", upload: "업로드", cache: "변환 캐시" }[m.dir] || m.dir}, ${m.size_mb} MB</option>`).join("");
}
on("#btnConf", async () => { await api("/api/settings/detection", { conf: +$("#confPct").value / 100 }); toast(`최소 확신도 ${$("#confPct").value}%로 저장했습니다`, "ok"); });
on("#btnModel", async () => { await api("/api/models/select", { name: $("#modelSel").value }); toast("모델을 바꾸는 중…", "info"); });
on("#btnUpload", async () => {
  const f = $("#modelFile").files[0]; if (!f) return toast("먼저 파일을 고르세요", "warn");
  const fd = new FormData(); fd.append("file", f);
  await api("/api/models/upload", fd); toast("업로드했습니다", "ok"); loadModels();
});

// ---------------------------------------------------------------- events
async function loadEvents() {
  const r = await api("/api/events?limit=150", undefined, "GET").catch(() => null); if (!r) return;
  let evs = r.events;
  if (evFilter === "alarm") evs = evs.filter((e) => e.level === "alarm");
  if (evFilter === "unacked") evs = evs.filter((e) => !e.acked);
  $("#eventList").innerHTML = evs.map((e) => {
    const d = new Date(e.t * 1000), label = EVENT_LABEL[e.type] || e.type;
    const clip = ["ready", "recording", "encoding"].includes(e.clip_status) ? " · 영상 있음(영상 메뉴)" : "";
    return `<div class="ev ${e.level} ${e.acked ? "" : "unacked"}"><span class="stripe"></span><div>` +
      `<div class="title">${esc(label)}</div><div class="msg">${esc(e.message)}</div>` +
      `<div class="meta">${d.toLocaleString("ko-KR")} · 당시 상태 ${esc(STATE_LABEL[e.state] || e.state || "—")}${e.acked ? ` · ${esc(e.acked_by)} 확인` : ""}${clip}</div></div>` +
      `<div class="acts">${e.acked ? "" : `<button class="btn ghost" data-ack="${e.id}">확인</button>`}</div></div>`;
  }).join("") || '<div class="ev-empty">이벤트가 없습니다</div>';
}
$("#eventList").addEventListener("click", async (e) => {
  const b = e.target.closest("button"); if (!b) return;
  if (b.dataset.ack) { await api(`/api/events/${b.dataset.ack}/ack`); loadEvents(); }
});
on("#playerClose", () => { $("#playerVideo").pause(); $("#player").classList.add("hidden"); });
$$("#evFilter button").forEach((b) => b.addEventListener("click", () => {
  evFilter = b.dataset.f; $$("#evFilter button").forEach((x) => x.classList.toggle("active", x === b)); loadEvents();
}));
on("#btnAckAll", async () => { await api("/api/events/ack_all"); loadEvents(); });
on("#btnNotif", async () => {
  if (!window.Notification) return toast("이 브라우저는 알림을 지원하지 않습니다 (휴대폰은 보통 HTTPS가 필요)", "warn");
  toast("알림 권한: " + ({ granted: "허용", denied: "거부", default: "보류" }[await Notification.requestPermission()] || ""));
});

// ---------------------------------------------------------------- videos (분류별 사고 영상)
let vidCat = "human", vidRows = [];
const VID_HINT = {
  human: "사람이 정지 거리 안으로 들어와 로봇이 멈춘 순간의 영상입니다 (멈추기 전·후 포함).",
  conveyor: "컨베이어가 돌기 시작한 뒤 제한 시간 안에 도착 센서에 물품이 오지 않은 경우(컨베이어 이상)의 영상입니다.",
  fire: "화재 감지부터 관리자가 종료할 때까지의 영상입니다. 저장 공간이 모자라도 지우지 않습니다.",
  other: "사람 사라짐·비상정지·고장으로 멈춘 영상입니다.",
};
function fmtBytes(b) { return b ? (b > 1e6 ? `${(b / 1e6).toFixed(1)} MB` : `${Math.round(b / 1e3)} KB`) : ""; }
async function loadVideos(more = false) {
  const day = $("#vidDay").value;
  const q = new URLSearchParams({ cat: vidCat, limit: 60 });
  if (day) q.set("day", day);
  if (more && vidRows.length) q.set("before", vidRows[vidRows.length - 1].id);
  const r = await api("/api/clips?" + q, undefined, "GET").catch(() => null); if (!r) return;
  vidRows = more ? vidRows.concat(r.clips) : r.clips;
  let unacked = 0;
  for (const [c, v] of Object.entries(r.counts)) {
    $("#vc-" + c).textContent = v.n ? ` ${v.n}` : "";
    $("#vc-" + c).classList.toggle("hot", v.unacked > 0);
    unacked += v.unacked;
  }
  $("#vidUnacked").textContent = unacked; $("#vidUnacked").classList.toggle("hidden", !unacked);
  $("#vidHint").textContent = VID_HINT[vidCat];
  $("#vidMore").classList.toggle("hidden", r.clips.length < 60);
  let lastDay = "", html = "";
  for (const e of vidRows) {
    const d = new Date(e.t * 1000), dk = d.toLocaleDateString("ko-KR", { year: "numeric", month: "long", day: "numeric", weekday: "short" });
    if (dk !== lastDay) { html += `<div class="vid-day">${dk}</div>`; lastDay = dk; }
    const label = EVENT_LABEL[e.type] || e.type;
    const st = e.clip_status === "ready" ? "" : e.clip_status === "failed" ? '<span class="tag bad">영상 실패</span>' : '<span class="tag warn">녹화 중…</span>';
    const ready = e.clip_status === "ready";
    html += `<div class="ev vid ${e.acked ? "" : "unacked"} ${e.level}"><span class="stripe"></span>` +
      `<button class="vid-play" ${ready ? `data-play="${esc(e.clip)}" data-title="${esc(label)} · ${d.toLocaleString("ko-KR")}"` : "disabled"} title="재생">` +
      `<svg><use href="#i-play"/></svg></button><div>` +
      `<div class="title">${d.toLocaleTimeString("ko-KR")} <span class="muted small">${vidCat === "other" ? esc(label) : ""}</span></div>` +
      `<div class="msg">${esc(e.message)}</div>` +
      `<div class="meta">당시 상태 ${esc(STATE_LABEL[e.state] || e.state || "—")}${ready ? " · " + fmtBytes(e.clip_bytes) : ""}` +
      `${e.acked ? ` · ${esc(e.acked_by)} 확인` : " · 미확인"}</div></div>` +
      `<div class="acts">${st}${ready ? `<a class="btn ghost" href="/clips/${encodeURIComponent(e.clip)}?dl=1" download>저장</a>` : ""}` +
      `${e.acked ? "" : `<button class="btn ghost" data-ack="${e.id}">확인</button>`}</div></div>`;
  }
  $("#vidList").innerHTML = html || `<div class="ev-empty">${day ? "이 날짜에 " : ""}영상이 없습니다</div>`;
}
$("#vidList").addEventListener("click", async (e) => {
  const b = e.target.closest("button"); if (!b) return;
  if (b.dataset.play) {
    $("#playerTitle").textContent = b.dataset.title; $("#playerVideo").src = "/clips/" + encodeURIComponent(b.dataset.play);
    $("#player").classList.remove("hidden"); $("#playerVideo").play().catch(() => {});
  }
  if (b.dataset.ack) { await api(`/api/events/${b.dataset.ack}/ack`); loadVideos(); }
});
$$("#vidCat button").forEach((b) => b.addEventListener("click", () => {
  vidCat = b.dataset.c; $$("#vidCat button").forEach((x) => x.classList.toggle("active", x === b)); loadVideos();
}));
$("#vidDay").addEventListener("change", () => loadVideos());
on("#vidDayAll", () => { $("#vidDay").value = ""; loadVideos(); });
on("#vidMore", () => loadVideos(true));

// ---------------------------------------------------------------- jog (moves only while held)
// 관절 조그(기본): J1~J5·그리퍼를 하나씩 직접. 좌표 조그: 그리퍼 끝을 X·Y·Z·기울기로.
let jogMode = "joint";
const KEYMAP = {
  joint: { q: "0,1", a: "0,-1", w: "1,1", s: "1,-1", e: "2,1", d: "2,-1", r: "3,1", f: "3,-1", t: "4,1", g: "4,-1", o: "5,1", c: "5,-1" },
  cart: { w: "0,1", s: "0,-1", a: "1,1", d: "1,-1", r: "2,1", f: "2,-1", t: "3,1", g: "3,-1" },
};
const held = new Set();
function jogVec() {
  const v = jogMode === "joint" ? [0, 0, 0, 0, 0, 0] : [0, 0, 0, 0, 0];
  held.forEach((k) => { const [i, s] = k.split(",").map(Number); if (i < v.length) v[i] += s; });
  return v.map((x) => Math.max(-1, Math.min(1, x)));
}
setInterval(() => { if (held.size && ws && ws.readyState === 1) ws.send(JSON.stringify({ jog: jogVec(), mode: jogMode })); }, 80);
const typing = () => { const a = document.activeElement; return a && ((a.tagName === "INPUT" && !["checkbox", "radio", "range"].includes(a.type)) || a.tagName === "SELECT" || a.tagName === "TEXTAREA"); };
document.addEventListener("keydown", (e) => {
  if (!$("#kbJog").checked || page !== "teach" || typing()) return;
  const m = KEYMAP[jogMode][e.key.toLowerCase()];
  if (m) { held.add(m); e.preventDefault(); markPad(); }
});
document.addEventListener("keyup", (e) => { const m = KEYMAP[jogMode][e.key.toLowerCase()]; if (m) { held.delete(m); markPad(); } });
window.addEventListener("blur", () => { held.clear(); markPad(); });
function markPad() {
  $$(".jog button").forEach((b) => b.classList.toggle("active", jogMode === "cart" && held.has(b.dataset.jog)));
  $$(".jj button").forEach((b) => b.classList.toggle("active", jogMode === "joint" && held.has(b.dataset.jj)));
}
$$(".jog button, .jj button").forEach((b) => {
  const key = b.dataset.jog || b.dataset.jj;
  const down = (e) => { e.preventDefault(); held.add(key); markPad(); b.setPointerCapture?.(e.pointerId); };
  const up = () => { held.delete(key); markPad(); };
  b.addEventListener("pointerdown", down);
  ["pointerup", "pointercancel", "pointerleave"].forEach((ev) => b.addEventListener(ev, up));
});
$$("#jogMode button").forEach((b) => b.addEventListener("click", () => {
  held.clear(); jogMode = b.dataset.m; markPad();
  $$("#jogMode button").forEach((x) => x.classList.toggle("active", x === b));
  $("#jogJoint").classList.toggle("hidden", jogMode !== "joint");
  $("#jogCart").classList.toggle("hidden", jogMode !== "cart");
  $("#jogHint").textContent = jogMode === "joint"
    ? "누르고 있는 동안만 움직입니다. 막대 = 관절 가동 범위 안의 현재 위치(각도). 키보드: 관절마다 위 키 +, 아래 키 −."
    : "그리퍼 끝을 로봇 베이스 기준으로 움직입니다(키보드 W/S 앞뒤, A/D 좌우, R/F 위아래, T/G 기울기). 5축이라 좌우는 베이스 회전으로 움직입니다.";
}));
$("#kbJog").addEventListener("change", (e) => $("#jogJoint").classList.toggle("kb", e.target.checked));
$("#jogSpeed").addEventListener("change", (e) => api("/api/robot/speed", { ratio: e.target.value / 100 }).catch(() => {}));
$("#jogSpeed").addEventListener("input", (e) => { $("#jogSpeedTxt").textContent = `${e.target.value}%`; });
function renderJogBars(rb) {
  if (!rb.qmin) return;
  $$(".jj-row").forEach((row) => {
    const j = +row.dataset.j, q = rb.q[j], lo = rb.qmin[j], hi = rb.qmax[j];
    const f = Math.min(1, Math.max(0, (q - lo) / (hi - lo)));
    row.querySelector(".jbar i").style.left = `calc(${(f * 100).toFixed(1)}% - 2px)`;
    row.querySelector(".jbar b").textContent = j === 5 ? `${gripPctOf(q) ?? "—"}% 열림` : `${(q * 180 / Math.PI).toFixed(1)}°`;
    row.classList.toggle("edge", f < 0.03 || f > 0.97);
  });
  const sp = $("#jogSpeed");
  if (document.activeElement !== sp) { sp.max = Math.round(rb.speed_cap * 100); sp.value = Math.round(rb.speed_ratio * 100); }
  if (document.activeElement !== sp) $("#jogSpeedTxt").textContent = `${Math.round(rb.speed_ratio * 100)}%`;
}

// ---------------------------------------------------------------- camera clicks (calibration / exit zone)
function startClick(mode, hint) {
  clickMode = mode; roiClicks = [];
  $("#cam").classList.add("clicking"); $("#camHint").textContent = hint; $("#camHint").classList.remove("hidden");
}
function endClick() { clickMode = null; $("#cam").classList.remove("clicking"); $("#camHint").classList.add("hidden"); }
function videoXY(e) {
  const img = $("#video"), rect = img.getBoundingClientRect();
  const nw = img.naturalWidth || 640, nh = img.naturalHeight || 480;
  const sc = Math.min(rect.width / nw, rect.height / nh);
  return [(e.clientX - rect.left - (rect.width - nw * sc) / 2) / sc, (e.clientY - rect.top - (rect.height - nh * sc) / 2) / sc];
}
$("#video").addEventListener("click", async (e) => {
  if (!clickMode) return;
  const [u, v] = videoXY(e);
  if (clickMode === "calib") {
    // 실패(깊이값 없음 등)하면 클릭 모드를 유지해 바로 다시 찍을 수 있게 한다
    try { const r = await api("/api/calib/sample", { u, v }); endClick(); toast(`보정 점 ${r.result}개째 추가 — 팔을 다른 자세로 옮긴 뒤 다시 추가하세요 (4개 이상)`, "ok", 7000); }
    catch (_) { if (clickMode === "calib") $("#camHint").textContent = "다시 클릭하세요 (그리퍼 끝 바로 옆, 몸체 쪽)"; }
  } else if (clickMode === "roi") {
    roiClicks.push([u, v]);
    if (roiClicks.length === 1) { $("#camHint").textContent = "반대쪽 모서리를 클릭하세요"; return; }
    const [[x1, y1], [x2, y2]] = roiClicks; endClick();
    await api("/api/settings/door_roi", { roi: [x1, y1, x2, y2] }); toast("출구 영역을 저장했습니다", "ok");
  }
});
on("#btnRoiSet", () => startClick("roi", "출구 영역의 첫 모서리를 클릭하세요"));
on("#btnRoiClear", () => api("/api/settings/door_roi", { roi: null }));
on("#btnCalClick", () => startClick("calib", "그리퍼 끝을 클릭하세요"));
on("#btnCalAruco", async () => { const r = await api("/api/calib/sample", {}); toast(`보정 점 ${r.result}개째 추가`, "ok"); });
on("#btnCalAuto", async () => {
  if (!confirm("로봇이 보정 자세 8곳을 차례로 움직입니다. 주변이 비어 있나요?")) return;
  await api("/api/calib/auto"); toast("자동 보정을 시작했습니다", "info");
});
on("#btnCalSolve", async () => { const r = await api("/api/calib/solve"); toast(`보정 완료 — 오차 ${(r.result.mean * 1000).toFixed(1)} mm`, "ok"); });
on("#btnCalReset", async () => { if (confirm("수동으로 찍은 보정 점을 지울까요? (보정 결과는 그대로 둡니다)")) await api("/api/calib/reset"); });
on("#btnCalDelete", async () => {
  if (!confirm("보정 결과를 삭제할까요?\n삭제하면 사람 거리를 로봇이 아니라 카메라 기준으로 재게 되어, 거리 기준이 제대로 동작하지 않습니다.")) return;
  await api("/api/calib/delete"); toast("보정 결과를 삭제했습니다 (백업: data/calib_history)", "warn", 8000);
});

// ---------------------------------------------------------------- PWA
if ("serviceWorker" in navigator) navigator.serviceWorker.register("/sw.js").catch(() => {});
boot();


// ------------------------------------------------------------------ 시각화 (SVG, 외부 라이브러리 없음)
const VC = { stop: "#dc2626", slow: "#f59e0b", hold: "#fcd34d", clear: "#16a34a", arm: "#1e3a8a", grid: "#e3e8f0", text: "#5d6a80", ink: "#13213c" };

// 거리 기준 막대: 정지(빨강) · 감속(주황) · 재가동 대기(노랑) · 여유(초록), 지금 가장 가까운 사람 위치 표시
function drawGauge(el, th, d, zone, preview = false) {
  if (!el || !th) return;
  const bad = !(th.stop < th.slow && th.slow <= th.clear);
  const max = Math.max(th.clear * 1.6, th.clear + 0.05), W = 400, x0 = 8, x1 = W - 8, X = (m) => x0 + (x1 - x0) * Math.min(1, Math.max(0, m / max));
  const seg = (a, b, c, lbl, ink = "#fff") => `<rect x="${X(a)}" y="22" width="${Math.max(0, X(b) - X(a))}" height="14" fill="${c}"/>` +
    (X(b) - X(a) > 34 ? `<text x="${(X(a) + X(b)) / 2}" y="32.5" class="seg" fill="${ink}">${lbl}</text>` : "");
  const cid = `${el.id}Clip`;
  const tick = (m, t) => `<line x1="${X(m)}" y1="20" x2="${X(m)}" y2="40" stroke="${VC.ink}" stroke-width="1"/><text x="${X(m)}" y="52" class="tick">${t}</text>`;
  let mk = "";
  if (d !== null && d !== undefined) {
    const x = X(d), col = d < th.stop ? VC.stop : d < th.slow ? VC.slow : d <= th.clear ? "#b45309" : VC.clear;
    mk = `<path d="M${x - 6},6 L${x + 6},6 L${x},17 Z" fill="${col}"/><line x1="${x}" y1="17" x2="${x}" y2="38" stroke="${col}" stroke-width="2.5"/>` +
      `<text x="${Math.min(x1 - 30, Math.max(x0 + 30, x))}" y="4" class="val" fill="${col}">${d > max ? "> " : ""}${fx(d, 2)} m</text>`;
  }
  el.innerHTML = `<svg viewBox="0 -8 ${W} 64" preserveAspectRatio="xMidYMid meet" role="img" aria-label="거리 기준">
    <defs><clipPath id="${cid}"><rect x="${x0}" y="22" width="${x1 - x0}" height="14" rx="7"/></clipPath></defs>
    <rect x="${x0}" y="22" width="${x1 - x0}" height="14" rx="7" fill="${VC.grid}"/>
    <g clip-path="url(#${cid})">${seg(0, th.stop, VC.stop, "정지")}${seg(th.stop, th.slow, VC.slow, "감속")}${seg(th.slow, th.clear, VC.hold, "대기", "#78350f")}${seg(th.clear, max, "#bbf7d0", "여유", "#166534")}</g>
    ${tick(0, "0")}${tick(th.stop, th.stop)}${tick(th.slow, th.slow)}${tick(th.clear, th.clear)}<text x="${x1}" y="52" class="tick end">m</text>
    ${mk}${d === null || d === undefined ? `<text x="${W / 2}" y="4" class="val muted">${preview ? "미리 보기" : "사람 없음"}</text>` : ""}</svg>` +
    (bad ? `<div class="err small">정지 &lt; 감속 ≤ 재가동 허용 순서여야 합니다</div>` : "");
}

// 감시 구역 지도 (위에서 본 모습): 위 = 로봇 앞(+x), 왼쪽 = +y. 단위 cm.
function drawZoneMap(el, m, th) {
  if (!el || !m || !m.zone) return;
  const z = m.zone, cm = (v) => v * 100;
  const W = cm(z.y[1] - z.y[0]), H = cm(z.x[1] - z.x[0]);
  const P = (p) => [cm(z.y[1] - p[1]), cm(z.x[1] - p[0])];       // 로봇 베이스 (x, y) → 화면
  let g = "";
  for (let v = Math.ceil(z.y[0] * 4) / 4; v <= z.y[1]; v += 0.25) { const [x] = P([0, v]); g += `<line x1="${x}" y1="0" x2="${x}" y2="${H}" class="gl${Math.abs(v) < 1e-6 ? " axis" : ""}"/>`; }
  for (let v = Math.ceil(z.x[0] * 4) / 4; v <= z.x[1]; v += 0.25) { const [, y] = P([v, 0]); g += `<line x1="0" y1="${y}" x2="${W}" y2="${y}" class="gl${Math.abs(v) < 1e-6 ? " axis" : ""}"/>`; }
  let arm = "", ppl = "";
  const links = m.links;
  if (links && links.length > 1) {
    const pts = links.map(P).map(([x, y]) => `${x.toFixed(1)},${y.toFixed(1)}`).join(" ");
    arm = `<polyline points="${pts}" class="band" stroke="${VC.slow}" stroke-width="${cm(th.slow) * 2}"/>` +
      `<polyline points="${pts}" class="band" stroke="${VC.stop}" stroke-width="${cm(th.stop) * 2}"/>` +
      `<polyline points="${pts}" class="arm"/>`;
    const tcp = P(links[links.length - 1]);
    arm += `<circle cx="${tcp[0]}" cy="${tcp[1]}" r="2.2" fill="#a21caf"/>`;
  }
  const [bx, by] = P([0, 0]);
  arm += `<rect x="${bx - 4}" y="${by - 4}" width="8" height="8" rx="2" fill="${VC.arm}"/>`;
  for (const p of (m.persons || [])) {
    if (!p.pos) continue;
    const [x, y] = P(p.pos), d = p.dist;
    const col = d === null ? VC.text : d < th.stop ? VC.stop : d < th.slow ? VC.slow : VC.clear;
    ppl += `<g><circle cx="${x}" cy="${y}" r="3.2" fill="${p.fallen ? "#fff" : col}" stroke="${col}" stroke-width="${p.fallen ? 1.6 : 1}"/>` +
      `<text x="${x}" y="${y - 5}" class="plbl" fill="${col}">${p.fallen ? "넘어짐 " : ""}${d === null ? "?" : fx(d, 2) + "m"}</text></g>`;
  }
  const note = !links ? `<text x="${W / 2}" y="${H / 2}" class="note">카메라-로봇 보정 후 표시됩니다</text>` : "";
  el.innerHTML = `<svg viewBox="-2 -2 ${W + 4} ${H + 4}" preserveAspectRatio="xMidYMid meet" role="img" aria-label="감시 구역 지도">
    <rect x="0" y="0" width="${W}" height="${H}" rx="3" class="zone"/>${g}${arm}${ppl}${note}
    <text x="${W - 2}" y="${H - 2}" class="scale">칸 25 cm</text><text x="${bx}" y="${by + 11}" class="scale mid">로봇</text></svg>`;
}

// 컨베이어 이동 시간 차트: 최근 도착까지 걸린 시간 막대 + 제한시간 선 + 지금 가동 중이면 진행 막대
function drawConvChart(el, cv) {
  if (!el) return;
  const h = cv.history || [], to = cv.timeout || 0, live = cv.timing && cv.running_for != null ? cv.running_for : null;
  if (!h.length && live === null) {
    el.innerHTML = `<div class="chart-empty">아직 기록이 없습니다 — 물품이 도착 센서에 닿을 때마다 걸린 시간이 쌓입니다</div>`;
    return;
  }
  const W = 400, H = 150, L = 30, R = 8, T = 10, B = 22;
  const vals = h.map((r) => r.s).concat(live !== null ? [live] : []);
  const top = Math.max(to || 0, ...vals, 1) * 1.15, Y = (v) => T + (H - T - B) * (1 - v / top);
  const n = 20, bw = (W - L - R) / n;
  let s = "";
  const step = top > 30 ? 10 : top > 12 ? 5 : top > 6 ? 2 : 1;
  for (let v = 0; v <= top; v += step) s += `<line x1="${L}" y1="${Y(v)}" x2="${W - R}" y2="${Y(v)}" class="gl"/><text x="${L - 4}" y="${Y(v) + 3.5}" class="tick end">${v}</text>`;
  const bars = h.slice(-(live !== null ? n - 1 : n));
  bars.forEach((r, i) => {
    const x = L + i * bw + bw * 0.18, w = bw * 0.64, y = Y(r.s);
    const tm = new Date(r.t * 1000).toTimeString().slice(0, 8);
    s += `<rect x="${x}" y="${y}" width="${w}" height="${H - B - y}" rx="2" fill="${r.jam ? VC.stop : "#3a5bef"}"><title>${tm} · ${r.jam ? "이상 (도착 안 함)" : fx(r.s, 1) + "초"}</title></rect>`;
  });
  if (live !== null) {
    const i = bars.length, x = L + i * bw + bw * 0.18, w = bw * 0.64, y = Y(live);
    s += `<rect x="${x}" y="${y}" width="${w}" height="${H - B - y}" rx="2" fill="#93c5fd" class="pulse"><title>지금 가동 중 ${fx(live, 1)}초</title></rect>`;
  }
  if (to > 0) s += `<line x1="${L}" y1="${Y(to)}" x2="${W - R}" y2="${Y(to)}" stroke="${VC.stop}" stroke-dasharray="5 4" stroke-width="1.5"/><text x="${W - R}" y="${Y(to) - 4}" class="tick end" fill="${VC.stop}">제한 ${to}초</text>`;
  const ok = h.filter((r) => !r.jam).map((r) => r.s), avg = ok.length ? ok.reduce((a, b) => a + b, 0) / ok.length : null;
  if (avg !== null) s += `<line x1="${L}" y1="${Y(avg)}" x2="${W - R}" y2="${Y(avg)}" stroke="${VC.clear}" stroke-width="1.2"/><text x="${W - R - 60}" y="${Y(avg) - 4}" class="tick end" fill="${VC.clear}">평균 ${fx(avg, 1)}초</text>`;
  s += `<text x="${L}" y="${H - 6}" class="tick">오래전</text><text x="${W - R}" y="${H - 6}" class="tick end">최근</text><text x="4" y="${T + 2}" class="tick">초</text>`;
  el.innerHTML = `<svg viewBox="0 0 ${W} ${H}" preserveAspectRatio="xMidYMid meet" role="img" aria-label="컨베이어 도착 시간">${s}</svg>`;
}
