// Connect to rosbridge WebSocket server
const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

// 연결 상태 표시
ros.on("connection", () => {
    document.getElementById("status").innerText = "successful";
});
ros.on("error", (error) => {
    document.getElementById("status").innerText = `errored out (${error})`;
});
ros.on("close", () => {
    document.getElementById("status").innerText = "closed";
});

let robot1 = 0, robot2 = 0, robot3 = 0, robot4 = 0;
let tab_Id = '';
// ============================================================
// 1번 로봇 관련 코드
// ============================================================
// ROS 토픽 구독 설정
const robot1Listener = new ROSLIB.Topic({
    ros,
    name: "/robot1receiver",
    messageType: "web_control_bridge/Robot1receiverMsg",
});

let robotX_1 = 275;
let robotY_1 = 200;
let ballX_1 = 0;
let ballY_1 = 0;
let currentYaw_1 = 0;

let imu_manager1 = 0;

robot1Listener.subscribe((message) => {
    robot1 = message.robot1;
    const pitch = message.pitch;
    const roll = message.roll;
    const yaw = message.yaw;

    robotX_1 = message.robot_x * 0.5;
    robotY_1 = message.robot_y * 0.5;
    ballX_1 = message.ball_x * 0.5;
    ballY_1 = message.ball_y * 0.5;

    const ik_x = message.x;
    const ik_y = message.y;

    const ball_flag = message.ball_flag;

    if (tab_Id === 'Data-Integration') {
        draw();
    }
    if (tab_Id === 'robot1') {
        draw1();
    }

    document.getElementById("orientation1").innerText =
        `Roll: ${roll.toFixed(1)}°, Pitch: ${pitch.toFixed(1)}°, Yaw: ${yaw.toFixed(1)}°`;

    document.getElementById("Walking1").innerText = `X: ${ik_x.toFixed(1)}, Y: ${ik_y.toFixed(1)}`;

    updateYawDial1(yaw);
    updateYawOnCanvas1(yaw + 90);

    const ballFlagElement = document.getElementById("ballFlag1");

    if (ball_flag === 1) {
        ballFlagElement.textContent = "Y";
        ballFlagElement.classList.add("active");
        ballFlagElement.classList.remove("inactive");
    }
    else {
        ballFlagElement.textContent = "N";
        ballFlagElement.classList.add("inactive");
        ballFlagElement.classList.remove("active");
    }
});

function updateYawDial1(yaw) {
    const pointer = document.getElementById("yaw-pointer1");
    const angle = yaw * (-1);
    pointer.style.transform = `rotate(${angle}deg)`;

    currentYaw_1 = yaw;
    if (tab_Id === 'Data-Integration') {
        draw();
    }
    if (tab_Id === 'robot1') {
        draw1();
    }
}

const publisher1 = new ROSLIB.Topic({
    ros: ros,
    name: "/robot1sender",
    messageType: "web_control_bridge/Robot1senderMsg"
});

const DEFAULT1 = { imu: -1, vision: -1, set: -1 };

// 부분 변경 → 상태 머지 → 전체 메시지 발행
function publish1(partial) {
    const msg = new ROSLIB.Message({ ...DEFAULT, ...partial });
    publisher1.publish(msg);   // 상태 저장 자체가 없으니 리셋 불필요
}

// 버튼 핸들러
const imu_on1 = document.getElementById('imu-on1');
const imu_off1 = document.getElementById('imu-off1');

function ImuOnManager1() {
    imu_on1.classList.add('active-on');
    imu_off1.classList.remove('active-off');
    publish1({ imu: 1 });
}

function ImuOffManager1() {
    imu_on1.classList.remove('active-on');
    imu_off1.classList.add('active-off');
    publish1({ imu: 0 });
}

const yolo_on1 = document.getElementById('yolo-on1');
const yolo_off1 = document.getElementById('yolo-off1');

function YoloOnManager1() {
    yolo_on1.classList.add('active-on');
    yolo_off1.classList.remove('active-off');
    publish1({ vision: 1 });
}

function YoloOffManager1() {
    yolo_on1.classList.remove('active-on');
    yolo_off1.classList.add('active-off');
    publish1({ vision: 0 });
}

function sendImuMessage1(setValue) {
    publish1({ set: setValue });
}

const arrowLength1 = 15;

// 캔버스 관련 변수
let ctx1, img1, canvas1

function draw1() {
    drawRobot(ctx1, img1, robotX_1, robotY_1, currentYaw_1, ballX_1, ballY_1, arrowLength1);
}

// 외부에서 yaw 업데이트 시 draw() 호출
function updateYawOnCanvas1(yawDeg) {
    currentYaw_1 = yawDeg;
    if (tab_Id === 'Data-Integration') {
        draw();
    }
    if (tab_Id === 'robot1') {
        draw1();
    }
}
// ============================================================

//============================================================
// 2번 로봇 관련 코드
// ============================================================
// ROS 토픽 구독 설정
const robot2Listener = new ROSLIB.Topic({
    ros,
    name: "/robot2receiver",
    messageType: "web_control_bridge/Robot2receiverMsg",
});

let robotX_2 = 275;
let robotY_2 = 200;
let ballX_2 = 0;
let ballY_2 = 0;
let currentYaw_2 = 0;

robot2Listener.subscribe((message) => {
    robot2 = message.robot2;
    const pitch = message.pitch;
    const roll = message.roll;
    const yaw = message.yaw;

    robotX_2 = message.robot_x * 0.5;
    robotY_2 = message.robot_y * 0.5;
    ballX_2 = message.ball_x * 0.5;
    ballY_2 = message.ball_y * 0.5;

    const ik_x = message.x;
    const ik_y = message.y;

    const ball_flag = message.ball_flag;

    if (tab_Id === 'Data-Integration') {
        draw();
    }
    if (tab_Id === 'robot2') {
        draw2();
    }

    document.getElementById("orientation2").innerText =
        `Roll: ${roll.toFixed(1)}°, Pitch: ${pitch.toFixed(1)}°, Yaw: ${yaw.toFixed(1)}°`;

    document.getElementById("Walking2").innerText = `X: ${message.x.toFixed(1)}, Y: ${message.y.toFixed(1)}`;

    updateYawDial2(yaw);
    updateYawOnCanvas2(yaw + 90);

    const ballFlagElement = document.getElementById("ballFlag2");

    if (ball_flag === 1) {
        ballFlagElement.textContent = "Y";
        ballFlagElement.classList.add("active");
        ballFlagElement.classList.remove("inactive");
    }
    else {
        ballFlagElement.textContent = "N";
        ballFlagElement.classList.add("inactive");
        ballFlagElement.classList.remove("active");
    }
});

function updateYawDial2(yaw) {
    const pointer = document.getElementById("yaw-pointer2");
    const angle = yaw * (-1);
    pointer.style.transform = `rotate(${angle}deg)`;

    currentYaw_2 = yaw;
    if (tab_Id === 'Data-Integration') {
        draw();
    }
    if (tab_Id === 'robot2') {
        draw2();
    }
}

const publisher2 = new ROSLIB.Topic({
    ros: ros,
    name: "/robot2sender",
    messageType: "web_control_bridge/Robot2senderMsg"
});

const DEFAULT2 = { imu: -1, vision: -1, set: -1 };

// 부분 변경 → 상태 머지 → 전체 메시지 발행
function publish2(partial) {
    const msg = new ROSLIB.Message({ ...DEFAULT, ...partial });
    publisher2.publish(msg);   // 상태 저장 자체가 없으니 리셋 불필요
}

// 버튼 핸들러
const imu_on2 = document.getElementById('imu-on2');
const imu_off2 = document.getElementById('imu-off2');

function ImuOnManager2() {
    imu_on2.classList.add('active-on');
    imu_off2.classList.remove('active-off');
    publish2({ imu: 1 });
}

function ImuOffManager2() {
    imu_on2.classList.remove('active-on');
    imu_off2.classList.add('active-off');
    publish2({ imu: 0 });
}

const yolo_on2 = document.getElementById('yolo-on2');
const yolo_off2 = document.getElementById('yolo-off2');

function YoloOnManager2() {
    yolo_on2.classList.add('active-on');
    yolo_off2.classList.remove('active-off');
    publish2({ vision: 1 });
}

function YoloOffManager2() {
    yolo_on2.classList.remove('active-on');
    yolo_off2.classList.add('active-off');
    publish2({ vision: 0 });
}

function sendImuMessage2(setValue) {
    publish2({ set: setValue });
}

const arrowLength2 = 15;

// 캔버스 관련 변수
let ctx2, img2, canvas2

function draw2() {
    drawRobot(ctx2, img2, robotX_2, robotY_2, currentYaw_2, ballX_2, ballY_2, arrowLength2);
}

// 외부에서 yaw 업데이트 시 draw() 호출
function updateYawOnCanvas2(yawDeg) {
    currentYaw_2 = yawDeg;
    if (tab_Id === 'Data-Integration') {
        draw();
    }
    if (tab_Id === 'robot2') {
        draw2();
    }
}
// ============================================================

//============================================================
// 3번 로봇 관련 코드
// ============================================================
// ROS 토픽 구독 설정
const robot3Listener = new ROSLIB.Topic({
    ros,
    name: "/robot3receiver",
    messageType: "web_control_bridge/Robot3receiverMsg",
});

let robotX_3 = 275;
let robotY_3 = 200;
let ballX_3 = 0;
let ballY_3 = 0;
let currentYaw_3 = 0;

robot3Listener.subscribe((message) => {
    robot3 = message.robot3;
    const pitch = message.pitch;
    const roll = message.roll;
    const yaw = message.yaw;

    robotX_3 = message.robot_x * 0.5;
    robotY_3 = message.robot_y * 0.5;
    ballX_3 = message.ball_x * 0.5;
    ballY_3 = message.ball_y * 0.5;

    const ik_x = message.x;
    const ik_y = message.y;

    const ball_flag = message.ball_flag;

    if (tab_Id === 'Data-Integration') {
        draw();
    }
    if (tab_Id === 'robot3') {
        draw3();
    }

    document.getElementById("orientation3").innerText =
        `Roll: ${roll.toFixed(1)}°, Pitch: ${pitch.toFixed(1)}°, Yaw: ${yaw.toFixed(1)}°`;

    document.getElementById("Walking3").innerText = `X: ${message.x.toFixed(1)}, Y: ${message.y.toFixed(1)}`;

    updateYawDial3(yaw);
    updateYawOnCanvas3(yaw + 90);

    const ballFlagElement = document.getElementById("ballFlag3");

    if (ball_flag === 1) {
        ballFlagElement.textContent = "Y";
        ballFlagElement.classList.add("active");
        ballFlagElement.classList.remove("inactive");
    }
    else {
        ballFlagElement.textContent = "N";
        ballFlagElement.classList.add("inactive");
        ballFlagElement.classList.remove("active");
    }
});

function updateYawDial3(yaw) {
    const pointer = document.getElementById("yaw-pointer3");
    const angle = yaw * (-1);
    pointer.style.transform = `rotate(${angle}deg)`;

    currentYaw_3 = yaw;
    if (tab_Id === 'Data-Integration') {
        draw();
    }
    if (tab_Id === 'robot3') {
        draw3();
    }
}

const publisher3 = new ROSLIB.Topic({
    ros: ros,
    name: "/robot3sender",
    messageType: "web_control_bridge/Robot3senderMsg"
});

const DEFAULT3 = { imu: -1, vision: -1, set: -1 };

// 부분 변경 → 상태 머지 → 전체 메시지 발행
function publish3(partial) {
    const msg = new ROSLIB.Message({ ...DEFAULT, ...partial });
    publisher3.publish(msg);   // 상태 저장 자체가 없으니 리셋 불필요
}

// 버튼 핸들러
const imu_on3 = document.getElementById('imu-on3');
const imu_off3 = document.getElementById('imu-off3');

function ImuOnManager3() {
    imu_on3.classList.add('active-on');
    imu_off3.classList.remove('active-off');
    publish2({ imu: 1 });
}

function ImuOffManager3() {
    imu_on3.classList.remove('active-on');
    imu_off3.classList.add('active-off');
    publish2({ imu: 0 });
}

const yolo_on3 = document.getElementById('yolo-on3');
const yolo_off3 = document.getElementById('yolo-off3');

function YoloOnManager3() {
    yolo_on3.classList.add('active-on');
    yolo_off3.classList.remove('active-off');
    publish3({ vision: 1 });
}

function YoloOffManager3() {
    yolo_on3.classList.remove('active-on');
    yolo_off3.classList.add('active-off');
    publish3({ vision: 0 });
}

function sendImuMessage3(setValue) {
    publish3({ set: setValue });
}

const arrowLength3 = 15;

// 캔버스 관련 변수
let ctx3, img3, canvas3

function draw3() {
    drawRobot(ctx3, img3, robotX_3, robotY_3, currentYaw_3, ballX_3, ballY_3, arrowLength3);
}

// 외부에서 yaw 업데이트 시 draw() 호출
function updateYawOnCanvas3(yawDeg) {
    currentYaw_3 = yawDeg;
    if (tab_Id === 'Data-Integration') {
        draw();
    }
    if (tab_Id === 'robot3') {
        draw3();
    }
}
// ============================================================

//============================================================
// 3번 로봇 관련 코드
// ============================================================
// ROS 토픽 구독 설정
const robot4Listener = new ROSLIB.Topic({
    ros,
    name: "/robot4receiver",
    messageType: "web_control_bridge/Robot4receiverMsg",
});

let robotX_4 = 275;
let robotY_4 = 200;
let ballX_4 = 0;
let ballY_4 = 0;
let currentYaw_4 = 0;

robot4Listener.subscribe((message) => {
    robot4 = message.robot4;
    const pitch = message.pitch;
    const roll = message.roll;
    const yaw = message.yaw;

    robotX_4 = message.robot_x * 0.5;
    robotY_4 = message.robot_y * 0.5;
    ballX_4 = message.ball_x * 0.5;
    ballY_4 = message.ball_y * 0.5;

    const ik_x = message.x;
    const ik_y = message.y;

    const ball_flag = message.ball_flag;

    if (tab_Id === 'Data-Integration') {
        draw();
    }
    if (tab_Id === 'robot4') {
        draw4();
    }

    document.getElementById("orientation4").innerText =
        `Roll: ${roll.toFixed(1)}°, Pitch: ${pitch.toFixed(1)}°, Yaw: ${yaw.toFixed(1)}°`;

    document.getElementById("Walking4").innerText = `X: ${message.x.toFixed(1)}, Y: ${message.y.toFixed(1)}`;

    updateYawDial4(yaw);
    updateYawOnCanvas4(yaw + 90);

    const ballFlagElement = document.getElementById("ballFlag4");

    if (ball_flag === 1) {
        ballFlagElement.textContent = "Y";
        ballFlagElement.classList.add("active");
        ballFlagElement.classList.remove("inactive");
    }
    else {
        ballFlagElement.textContent = "N";
        ballFlagElement.classList.add("inactive");
        ballFlagElement.classList.remove("active");
    }

});

function updateYawDial4(yaw) {
    const pointer = document.getElementById("yaw-pointer4");
    const angle = yaw * (-1);
    pointer.style.transform = `rotate(${angle}deg)`;

    currentYaw_4 = yaw;
    if (tab_Id === 'Data-Integration') {
        draw();
    }
    if (tab_Id === 'robot4') {
        draw4();
    }
}

const publisher4 = new ROSLIB.Topic({
    ros: ros,
    name: "/robot4sender",
    messageType: "web_control_bridge/Robot4senderMsg"
});

const DEFAULT4 = { imu: -1, vision: -1, set: -1 };

// 부분 변경 → 상태 머지 → 전체 메시지 발행
function publish4(partial) {
    const msg = new ROSLIB.Message({ ...DEFAULT, ...partial });
    publisher4.publish(msg);   // 상태 저장 자체가 없으니 리셋 불필요
}

// 버튼 핸들러
const imu_on4 = document.getElementById('imu-on4');
const imu_off4 = document.getElementById('imu-off4');

function ImuOnManager4() {
    imu_on4.classList.add('active-on');
    imu_off4.classList.remove('active-off');
    publish4({ imu: 1 });
}

function ImuOffManager4() {
    imu_on4.classList.remove('active-on');
    imu_off4.classList.add('active-off');
    publish4({ imu: 0 });
}

const yolo_on4 = document.getElementById('yolo-on4');
const yolo_off4 = document.getElementById('yolo-off4');

function YoloOnManager4() {
    yolo_on4.classList.add('active-on');
    yolo_off4.classList.remove('active-off');
    publish4({ vision: 1 });
}

function YoloOffManager4() {
    yolo_on4.classList.remove('active-on');
    yolo_off4.classList.add('active-off');
    publish4({ vision: 0 });
}

function sendImuMessage4(setValue) {
    publish4({ set: setValue });
}

const arrowLength4 = 15;

// 캔버스 관련 변수
let ctx4, img4, canvas4

function draw4() {
    drawRobot(ctx4, img4, robotX_4, robotY_4, currentYaw_4, ballX_4, ballY_4, arrowLength4);
}

// 외부에서 yaw 업데이트 시 draw() 호출
function updateYawOnCanvas4(yawDeg) {
    currentYaw_4 = yawDeg;
    if (tab_Id === 'Data-Integration') {
        draw();
    }
    if (tab_Id === 'robot4') {
        draw4();
    }
}
// ============================================================

// ============================================================
// 공통 함수
// ============================================================

// 캔버스 관련 변수
let ctx, img, canvas

const arrowLength = 15;

let colors = "";

function draw() {
    if (!ctx || !img) return;

    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.drawImage(img, 0, 0, canvas.width, canvas.height);

    if (robot1 != 0) {
        colors = "blue";
        drawData(ctx, img, robotX_1, robotY_1, currentYaw_1, ballX_1, ballY_1, arrowLength, colors);
    }
    if (robot2 != 0) {
        colors = "green";
        drawData(ctx, img, robotX_2, robotY_2, currentYaw_2, ballX_2, ballY_2, arrowLength, colors);
    }
    if (robot3 != 0) {
        colors = "orange";
        drawData(ctx, img, robotX_3, robotY_3, currentYaw_3, ballX_3, ballY_3, arrowLength, colors);
    }
    if (robot4 != 0) {
        colors = "purple";
        drawData(ctx, img, robotX_4, robotY_4, currentYaw_4, ballX_4, ballY_4, arrowLength, colors);
    }
}

function drawData(ctx, img, robotX, robotY, yawDeg, ballX, ballY, arrowLength, color) {
    if (!ctx || !img) return;

    // 로봇 위치
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(robotX, robotY, 10, 0, Math.PI * 2);
    ctx.fill();

    ctx.strokeStyle = "black";  // 테두리 색상
    ctx.lineWidth = 2;          // 테두리 두께
    ctx.stroke();               // 테두리 그리기

    const yawRad = yawDeg * Math.PI / 180;
    const endX = robotX + arrowLength * Math.cos(yawRad);
    const endY = robotY - arrowLength * Math.sin(yawRad);

    // 방향선
    ctx.strokeStyle = "black";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(robotX, robotY);
    ctx.lineTo(endX, endY);
    ctx.stroke();

    // 공 위치
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(ballX, ballY, 4, 0, Math.PI * 2);
    ctx.fill();
}

function drawRobot(ctx, img, robotX, robotY, yawDeg, ballX, ballY, arrowLength) {
    if (!ctx || !img) return;

    ctx.clearRect(0, 0, img.width, img.height);
    ctx.drawImage(img, 0, 0, img.width, img.height);

    ctx.fillStyle = "blue";
    ctx.beginPath();
    ctx.arc(robotX, robotY, 10, 0, Math.PI * 2);
    ctx.fill();

    const yawRad = yawDeg * Math.PI / 180;
    const endX = robotX + arrowLength * Math.cos(yawRad);
    const endY = robotY - arrowLength * Math.sin(yawRad);

    ctx.strokeStyle = "black";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(robotX, robotY);
    ctx.lineTo(endX, endY);
    ctx.stroke();

    ctx.fillStyle = "red";
    ctx.beginPath();
    ctx.arc(ballX, ballY, 4, 0, Math.PI * 2);
    ctx.fill();
}

window.onload = function () {
    // 탭 초기화
    const defaultTab = document.querySelector('.tab[href="#"][onclick*="robot1"]');
    showTab('robot1', defaultTab);

    // 캔버스 및 이미지 초기화
    img = document.getElementById("fieldImage");
    canvas = document.getElementById("fieldCanvas");
    ctx = canvas.getContext("2d");

    img1 = document.getElementById("fieldImage1");
    canvas1 = document.getElementById("fieldCanvas1");
    ctx1 = canvas1.getContext("2d");

    img2 = document.getElementById("fieldImage2");
    canvas2 = document.getElementById("fieldCanvas2");
    ctx2 = canvas2.getContext("2d");

    img3 = document.getElementById("fieldImage3");
    canvas3 = document.getElementById("fieldCanvas3");
    ctx3 = canvas3.getContext("2d");

    img4 = document.getElementById("fieldImage4");
    canvas4 = document.getElementById("fieldCanvas4");
    ctx4 = canvas4.getContext("2d");

    if (img.complete) {
        canvas.width = img.width;
        canvas.height = img.height;
        if (tab_Id === 'Data-Integration') {
            draw();
        }
    }
    else {
        img.onload = () => {
            canvas.width = img.width;
            canvas.height = img.height;
            if (tab_Id === 'Data-Integration') {
                draw();
            }
        };
    }

    if (img1.complete) {
        canvas1.width = img1.width;
        canvas1.height = img1.height;
        if (tab_Id === 'Data-Integration') {
            draw();
        }
        if (tab_Id === 'robot1') {
            draw1();
        }
    }
    else {
        img1.onload = () => {
            canvas1.width = img1.width;
            canvas1.height = img1.height;
            if (tab_Id === 'Data-Integration') {
                draw();
            }
            if (tab_Id === 'robot1') {
                draw1();
            }
        };
    }

    if (img2.complete) {
        canvas2.width = img2.width;
        canvas2.height = img2.height;
        if (tab_Id === 'Data-Integration') {
            draw();
        }
        if (tab_Id === 'robot2') {
            draw2();
        }
    }
    else {
        img2.onload = () => {
            canvas2.width = img2.width;
            canvas2.height = img2.height;
            if (tab_Id === 'Data-Integration') {
                draw();
            }
            if (tab_Id === 'robot2') {
                draw2();
            }
        };
    }

    if (img3.complete) {
        canvas3.width = img3.width;
        canvas3.height = img3.height;
        draw();
        if (tab_Id === 'Data-Integration') {
            draw();
        }
        if (tab_Id === 'robot3') {
            draw3();
        }
    }
    else {
        img3.onload = () => {
            canvas3.width = img3.width;
            canvas3.height = img3.height;
            if (tab_Id === 'Data-Integration') {
                draw();
            }
            if (tab_Id === 'robot3') {
                draw3();
            }
        };
    }

    if (img4.complete) {
        canvas4.width = img4.width;
        canvas4.height = img4.height;
        if (tab_Id === 'Data-Integration') {
            draw();
        }
        if (tab_Id === 'robot4') {
            draw4();
        }
    }
    else {
        img4.onload = () => {
            canvas4.width = img4.width;
            canvas4.height = img4.height;
            if (tab_Id === 'Data-Integration') {
                draw();
            }
            if (tab_Id === 'robot4') {
                draw4();
            }
        };
    }
};

function showTab(tabId, element) {
    // 탭 콘텐츠 숨기기
    const contents = document.querySelectorAll('.tab-content');
    contents.forEach(div => div.style.display = 'none');

    // 선택한 탭 콘텐츠 표시
    const activeTab = document.getElementById(tabId);
    if (activeTab) activeTab.style.display = 'block';

    // 버튼 스타일 업데이트
    document.querySelectorAll('.tab').forEach(btn => btn.classList.remove('active'));
    if (element) element.classList.add('active');

    tab_Id = tabId; // 현재 탭 ID 저장
}

// ============================================================