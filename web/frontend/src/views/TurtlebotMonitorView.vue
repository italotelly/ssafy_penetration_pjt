<template>
  <section>
    <div class="section-title-row">
      <div>
        <h2 class="section-title">TurtleBot</h2>
        <span class="text-muted">{{ nowText }}</span>
      </div>

      <span class="badge" :class="rosConnected ? 'badge-info' : 'badge-warn'">
        {{ rosConnected ? "LIVE" : "OFFLINE" }}
      </span>
    </div>

    <div class="tb-layout">
      <!-- LEFT -->
      <div class="tb-left">
        <div class="tb-cards">
          <!-- Battery -->
          <div class="content-card tb-card">
            <div class="tb-card-header">
              <span>Battery</span>
              <span class="text-muted">잔량 · 전압</span>
            </div>
            <div class="bar-row">
              <div class="bar-label">
                <span>State of Charge</span>
                <span class="text-muted">{{ socText }}</span>
              </div>
              <div class="bar-track">
                <div class="bar-fill" :style="{ width: minBar(socPct) + '%' }"></div>
              </div>
            </div>

            <div class="bar-row">
              <div class="bar-label">
                <span>Voltage</span>
                <span class="text-muted">{{ battery.voltage.toFixed(1) }} V</span>
              </div>
              <div class="bar-track">
                <div class="bar-fill" :style="{ width: minBar(voltPct) + '%' }"></div>
              </div>
            </div>

            <div class="bar-row">
              <div class="bar-label">
                <span>Current</span>
                <span class="text-muted">{{ battery.current.toFixed(1) }} A</span>
              </div>
              <div class="bar-track">
                <div class="bar-fill" :style="{ width: minBar(currPct) + '%' }"></div>
              </div>
            </div>
          </div>

          <!-- Motion -->
          <div class="content-card tb-card">
            <div class="tb-card-header">
              <span>Motion</span>
              <span class="text-muted">속도 · 방향</span>
            </div>
            <div class="bar-row">
              <div class="bar-label">
                <span>Linear Velocity</span>
                <span class="text-muted">{{ motion.lin.toFixed(2) }} m/s</span>
              </div>
              <div class="bar-track">
                <div class="bar-fill" :style="{ width: minBar(linPct) + '%' }"></div>
              </div>
            </div>

            <div class="bar-row">
              <div class="bar-label">
                <span>Angular Velocity</span>
                <span class="text-muted">{{ motion.ang.toFixed(2) }} rad/s</span>
              </div>
              <div class="bar-track">
                <div class="bar-fill" :style="{ width: minBar(angPct) + '%' }"></div>
              </div>
            </div>

            <div class="bar-row">
              <div class="bar-label">
                <span>Status</span>
                <span class="text-muted">{{ motion.status }}</span>
              </div>
            </div>
          </div>

          <!-- Wheel Joints -->
          <div class="content-card tb-card">
            <div class="tb-card-header">
              <span>Wheel Joints</span>
              <span class="text-muted">vel · effort</span>
            </div>
           <div class="bar-row">
              <div class="bar-label">
                <span>Left vel</span>
                <span class="text-muted">{{ joints.left.vel.toFixed(2) }}</span>
              </div>
              <div class="bar-track">
                <div class="bar-fill" :style="{ width: minBar(leftVelPct) + '%' }"></div>
              </div>
            </div>

            <div class="bar-row">
              <div class="bar-label">
                <span>Right vel</span>
                <span class="text-muted">{{ joints.right.vel.toFixed(2) }}</span>
              </div>
              <div class="bar-track">
                <div class="bar-fill" :style="{ width: minBar(rightVelPct) + '%' }"></div>
              </div>
            </div>

            <div class="bar-row">
              <div class="bar-label">
                <span>Effort(avg)</span>
                <span class="text-muted">{{ effortAvg.toFixed(2) }}</span>
              </div>
              <div class="bar-track">
                <div class="bar-fill" :style="{ width: minBar(effPct) + '%' }"></div>
              </div>
          </div>

          </div>
        </div>
      </div>

      <!-- RIGHT -->
      <div class="tb-right">
        <div class="content-card tb-map-card">
          <div class="tb-card-header">
            <span>Map</span>
            <span class="text-muted">현재 위치</span>
          </div>

          <div class="map-stage">
            <img class="map-img" :src="mapUrl" alt="slam map" ref="imgRef" @load="onMapLoaded" />
            <canvas class="map-overlay" ref="canvasRef"></canvas>
          </div>

          <div class="tb-map-footer text-muted">
            pose(world): x={{ poseWorld.x.toFixed(2) }}, y={{ poseWorld.y.toFixed(2) }}, yaw={{ poseWorld.yaw.toFixed(2) }}
          </div>
        </div>
      </div>
    </div>
  </section>
</template>

<script setup>
import { computed, onMounted, onBeforeUnmount, reactive, ref } from "vue";
import * as ROSLIB from "roslib";
import YAML from "js-yaml";

// 막대용 정규화(0~100)
const clamp01 = (v) => Math.min(1, Math.max(0, v));
const toPct = (v01) => Math.round(clamp01(v01) * 100);

// 배터리 기준값(원하면 바꿔)
const VOLT_MIN = 22.0;
const VOLT_MAX = 26.0;
const CURR_MAX_ABS = 5.0;      // |A|
const LIN_MAX = 0.35;          // m/s (turtlebot3 일반 주행 상한)
const ANG_MAX = 1.8;           // rad/s
const WHEEL_VEL_MAX = 10.0;    // rad/s (대충 상한)
const EFF_MAX = 1.0;           // effort 상한(환경마다 다름, 우선 1.0)

const socPct = computed(() => (battery.soc < 0 ? 0 : toPct(battery.soc)));

const voltPct = computed(() => {
  const v = (battery.voltage - VOLT_MIN) / (VOLT_MAX - VOLT_MIN);
  return toPct(v);
});

const currPct = computed(() => {
  const v = Math.abs(battery.current) / CURR_MAX_ABS;
  return toPct(v);
});

const linPct = computed(() => toPct(Math.abs(motion.lin) / LIN_MAX));
const angPct = computed(() => toPct(Math.abs(motion.ang) / ANG_MAX));

const leftVelPct = computed(() => toPct(Math.abs(joints.left.vel) / WHEEL_VEL_MAX));
const rightVelPct = computed(() => toPct(Math.abs(joints.right.vel) / WHEEL_VEL_MAX));

const effortAvg = computed(() => (Math.abs(joints.left.eff) + Math.abs(joints.right.eff)) / 2);
const effPct = computed(() => toPct(effortAvg.value / EFF_MAX));

// 0%일 때도 최소 4%는 보이게
function minBar(pct, min = 4) {
  if (!rosConnected.value) return min;   // OFFLINE이면 항상 최소
  return pct === 0 ? min : pct;
}


// 로봇 이미지 추가
const robotImg = new Image();
robotImg.src = "http://172.30.1.32:3000/public/icons/turtlebot.png";

let robotImgReady = false;
robotImg.onload = () => {
  robotImgReady = true;
};


/* ===============================
 * 0) 주소 설정
 * =============================== */
const mapUrl = "http://172.30.1.32:3000/public/maps/final.png";
const MAP_YAML_URL = "http://172.30.1.32:3000/public/maps/final.yaml";
const ROSBRIDGE_URL = "ws://172.30.1.69:9090";

/* ===============================
 * 1) 시간
 * =============================== */
const nowText = ref("");
let timerId = 0;

function pad2(n) {
  return String(n).padStart(2, "0");
}
function updateNow() {
  const d = new Date();
  nowText.value =
    `${d.getFullYear()}-${pad2(d.getMonth() + 1)}-${pad2(d.getDate())} ` +
    `${pad2(d.getHours())}:${pad2(d.getMinutes())}:${pad2(d.getSeconds())}`;
}

/* ===============================
 * 2) ROS 상태 값
 * =============================== */
const rosConnected = ref(false);

const battery = reactive({ soc: 0, voltage: 0, current: 0 });
const motion = reactive({ lin: 0, ang: 0, status: "Offline" });
const joints = reactive({
  left: { vel: 0, eff: 0 },
  right: { vel: 0, eff: 0 },
});
const poseWorld = reactive({ x: 0, y: 0, yaw: 0 });

const socText = computed(() =>
  battery.soc < 0 ? "N/A" : `${(battery.soc * 100).toFixed(0)} %`
);

/* ===============================
 * 3) Map Meta (YAML)
 * =============================== */
const mapMeta = reactive({
  resolution: 0.05,
  originX: -3.42,
  originY: -3.35,
  imgW: 0,
  imgH: 0,
  scaleX: 1,
  scaleY: 1,
  ready: false,
});

async function loadMapYaml() {
  const res = await fetch(MAP_YAML_URL);
  const y = YAML.load(await res.text());
  mapMeta.resolution = y.resolution;
  mapMeta.originX = y.origin[0];
  mapMeta.originY = y.origin[1];
  mapMeta.ready = true;
}

/* ===============================
 * 4) Canvas / Map Overlay
 * =============================== */
const imgRef = ref(null);
const canvasRef = ref(null);

function worldToCanvas(x, y) {
  if (!mapMeta.ready) return { px: 0, py: 0 };

  const u = (x - mapMeta.originX) / mapMeta.resolution;
  const v = (y - mapMeta.originY) / mapMeta.resolution;

  return {
    px: u * mapMeta.scaleX,
    py: (mapMeta.imgH - v) * mapMeta.scaleY,
  };
}

async function onMapLoaded() {
  const img = imgRef.value;
  const canvas = canvasRef.value;
  if (!img || !canvas) return;

  const rect = img.getBoundingClientRect();
  canvas.width = rect.width;
  canvas.height = rect.height;

  mapMeta.imgW = img.naturalWidth;
  mapMeta.imgH = img.naturalHeight;
  mapMeta.scaleX = canvas.width / mapMeta.imgW;
  mapMeta.scaleY = canvas.height / mapMeta.imgH;

  if (!mapMeta.ready) await loadMapYaml();
  drawOverlay();
}

function drawOverlay() {
  const canvas = canvasRef.value;
  if (!canvas) return;
  const ctx = canvas.getContext("2d");

  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const { px, py } = worldToCanvas(poseWorld.x, poseWorld.y);
  drawRobot(ctx, px, py, poseWorld.yaw);

  ctx.beginPath();
  ctx.arc(px, py, 4, 0, Math.PI * 2);
  ctx.fillStyle = "rgba(31,41,51,0.8)";
  ctx.fill();
}

function drawRobot(ctx, x, y, yaw) {
  if (!robotImgReady) return;

  const size = 32; // 화면에 보일 로봇 크기(px)

  ctx.save();
  
  // 1) 로봇 위치로 이동
  ctx.translate(x, y);

  // 2) 회전
  // PNG가 "위쪽이 전방" 기준이면 yaw - Math.PI / 2 필요 없음
  ctx.rotate(yaw);

  // 3) 이미지 중심 맞춰서 그리기
  ctx.drawImage(
    robotImg,
    -size,
    -size,
    size * 2,
    size * 2
  );

  ctx.restore();
}


function handleResize() {
  onMapLoaded();
}

/* ===============================
 * 5) ROS (rosbridge)
 * =============================== */
let ros, batteryTopic, odomTopic, jointTopic, amclTopic;

function quatToYaw(q) {
  return Math.atan2(
    2 * (q.w * q.z + q.x * q.y),
    1 - 2 * (q.y * q.y + q.z * q.z)
  );
}

function computeStatus(l, a) {
  if (Math.abs(l) < 0.01 && Math.abs(a) < 0.01) return "Stopped";
  if (Math.abs(a) > 0.25) return "Turning";
  return "Cruising";
}

function connectRos() {
  ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });

  ros.on("connection", () => {
    rosConnected.value = true;
    motion.status = "Connected";

    batteryTopic = new ROSLIB.Topic({
      ros,
      name: "/battery_state",
      messageType: "sensor_msgs/BatteryState",
    });
    batteryTopic.subscribe((m) => {
      battery.soc = m.percentage ?? -1;
      battery.voltage = m.voltage ?? 0;
      battery.current = m.current ?? 0;
    });

    odomTopic = new ROSLIB.Topic({
      ros,
      name: "/odom",
      messageType: "nav_msgs/Odometry",
    });
    odomTopic.subscribe((m) => {
      motion.lin = m.twist.twist.linear.x;
      motion.ang = m.twist.twist.angular.z;
      motion.status = computeStatus(motion.lin, motion.ang);
    });

    jointTopic = new ROSLIB.Topic({
      ros,
      name: "/joint_states",
      messageType: "sensor_msgs/JointState",
    });
    jointTopic.subscribe((m) => {
      const li = m.name.findIndex((n) => n.includes("left") && n.includes("wheel"));
      const ri = m.name.findIndex((n) => n.includes("right") && n.includes("wheel"));
      joints.left.vel = li >= 0 ? m.velocity[li] : 0;
      joints.left.eff = li >= 0 ? m.effort[li] : 0;
      joints.right.vel = ri >= 0 ? m.velocity[ri] : 0;
      joints.right.eff = ri >= 0 ? m.effort[ri] : 0;
    });

    amclTopic = new ROSLIB.Topic({
      ros,
      name: "/amcl_pose",
      messageType: "geometry_msgs/PoseWithCovarianceStamped",
    });
    amclTopic.subscribe((m) => {
      poseWorld.x = m.pose.pose.position.x;
      poseWorld.y = m.pose.pose.position.y;
      poseWorld.yaw = quatToYaw(m.pose.pose.orientation);
      drawOverlay();
    });
  });

  ros.on("close", () => (rosConnected.value = false));
}

function disconnectRos() {
  batteryTopic?.unsubscribe();
  odomTopic?.unsubscribe();
  jointTopic?.unsubscribe();
  amclTopic?.unsubscribe();
  ros?.close();
}

/* ===============================
 * 6) Lifecycle
 * =============================== */
onMounted(() => {
  updateNow();
  timerId = setInterval(updateNow, 1000);
  connectRos();
  window.addEventListener("resize", handleResize);
});

onBeforeUnmount(() => {
  clearInterval(timerId);
  disconnectRos();
  window.removeEventListener("resize", handleResize);
});
</script>


<style scoped>
.tb-layout {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 14px;
  align-items: start;
}

@media (max-width: 1100px) {
  .tb-layout {
    grid-template-columns: 1fr;
  }
}

.tb-cards {
  margin-top: 15px;
  display: grid;
  gap: 12px;
}

.tb-card {
  padding: 12px 14px 14px;
}

.tb-card-header {
  display: flex;
  justify-content: space-between;
  margin-bottom: 6px;
  font-size: 0.9rem;
  font-weight: 500;
}

.tb-map-card {
  margin-top: 50px;
  padding: 12px 14px 14px;
}

.map-stage {
  position: relative;
  width: 100%;
  border-radius: 10px;
  overflow: hidden;
  border: 1px solid #eef1f7;
  background: #fff;
}

.map-img {
  width: 100%;
  height: auto;
  display: block;
}

.map-overlay {
  position: absolute;
  inset: 0;
  width: 100%;
  height: 100%;
  pointer-events: none;
}

.tb-map-footer {
  margin-top: 8px;
  font-size: 0.82rem;
}
</style>
