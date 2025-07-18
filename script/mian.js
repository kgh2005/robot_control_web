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

// ============================================================
// 1번 로봇 관련 코드
// ============================================================
// ROS 토픽 구독 설정
const robot1Listener = new ROSLIB.Topic({
    ros,
    name: "/robot1receiver",
    messageType: "humanoid_interfaces/Robot1receiverMsg",
});

let robotX_1 = 275;
let robotY_1 = 200;
let ballX_1 = 0;
let ballY_1 = 0;
let currentYaw_1 = 0;

robot1Listener.subscribe((message) => {
    const pitch = message.pitch;
    const roll = message.roll;
    const yaw = message.yaw;

    robotX_1 = message.robot_x * 0.5;
    robotY_1 = message.robot_y * 0.5;
    ballX_1 = message.ball_x * 0.5;
    ballY_1 = message.ball_y * 0.5;

    draw1();

    document.getElementById("orientation1").innerText =
        `Roll: ${roll.toFixed(1)}°, Pitch: ${pitch.toFixed(1)}°, Yaw: ${yaw.toFixed(1)}°`;

    updateYawDial1(yaw);
    updateYawOnCanvas1(yaw + 90);
});

function updateYawDial1(yaw) {
    const pointer = document.getElementById("yaw-pointer1");
    const angle = yaw * (-1);
    pointer.style.transform = `rotate(${angle}deg)`;

    currentYaw_1 = yaw;
    draw1();
}

function sendTestMessage1(setValue) {
    const publisher = new ROSLIB.Topic({
        ros: ros,
        name: "/robot1sender",
        messageType: "humanoid_interfaces/Robot1senderMsg"
    });

    const msg = new ROSLIB.Message({ set: setValue });
    publisher.publish(msg);
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
    draw1();
}
// ============================================================

//============================================================
// 2번 로봇 관련 코드
// ============================================================
// ROS 토픽 구독 설정
const robot2Listener = new ROSLIB.Topic({
    ros,
    name: "/robot2receiver",
    messageType: "humanoid_interfaces/Robot2receiverMsg",
});

let robotX_2 = 275;
let robotY_2 = 200;
let ballX_2 = 0;
let ballY_2 = 0;
let currentYaw_2 = 0;

robot2Listener.subscribe((message) => {
    const pitch = message.pitch;
    const roll = message.roll;
    const yaw = message.yaw;

    robotX_2 = message.robot_x * 0.5;
    robotY_2 = message.robot_y * 0.5;
    ballX_2 = message.ball_x * 0.5;
    ballY_2 = message.ball_y * 0.5;

    draw2();

    document.getElementById("orientation2").innerText =
        `Roll: ${roll.toFixed(1)}°, Pitch: ${pitch.toFixed(1)}°, Yaw: ${yaw.toFixed(1)}°`;

    updateYawDial2(yaw);
    updateYawOnCanvas2(yaw + 90);
});

function updateYawDial2(yaw) {
    const pointer = document.getElementById("yaw-pointer2");
    const angle = yaw * (-1);
    pointer.style.transform = `rotate(${angle}deg)`;

    currentYaw_2 = yaw;
    draw2();
}

function sendTestMessage2(setValue) {
    const publisher = new ROSLIB.Topic({
        ros: ros,
        name: "/robot2sender",
        messageType: "humanoid_interfaces/Robot2senderMsg"
    });

    const msg = new ROSLIB.Message({ set: setValue });
    publisher.publish(msg);
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
    draw2();
}
// ============================================================

//============================================================
// 3번 로봇 관련 코드
// ============================================================
// ROS 토픽 구독 설정
const robot3Listener = new ROSLIB.Topic({
    ros,
    name: "/robot3receiver",
    messageType: "humanoid_interfaces/Robot3receiverMsg",
});

let robotX_3 = 275;
let robotY_3 = 200;
let ballX_3 = 0;
let ballY_3 = 0;
let currentYaw_3 = 0;

robot3Listener.subscribe((message) => {
    const pitch = message.pitch;
    const roll = message.roll;
    const yaw = message.yaw;

    robotX_3 = message.robot_x * 0.5;
    robotY_3 = message.robot_y * 0.5;
    ballX_3 = message.ball_x * 0.5;
    ballY_3 = message.ball_y * 0.5;

    draw3();

    document.getElementById("orientation3").innerText =
        `Roll: ${roll.toFixed(1)}°, Pitch: ${pitch.toFixed(1)}°, Yaw: ${yaw.toFixed(1)}°`;

    updateYawDial3(yaw);
    updateYawOnCanvas3(yaw + 90);
});

function updateYawDial3(yaw) {
    const pointer = document.getElementById("yaw-pointer3");
    const angle = yaw * (-1);
    pointer.style.transform = `rotate(${angle}deg)`;

    currentYaw_3 = yaw;
    draw3();
}

function sendTestMessage3(setValue) {
    const publisher = new ROSLIB.Topic({
        ros: ros,
        name: "/robot3sender",
        messageType: "humanoid_interfaces/Robot3senderMsg"
    });

    const msg = new ROSLIB.Message({ set: setValue });
    publisher.publish(msg);
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
    draw3();
}
// ============================================================

//============================================================
// 3번 로봇 관련 코드
// ============================================================
// ROS 토픽 구독 설정
const robot4Listener = new ROSLIB.Topic({
    ros,
    name: "/robot4receiver",
    messageType: "humanoid_interfaces/Robot4receiverMsg",
});

let robotX_4 = 275;
let robotY_4 = 200;
let ballX_4 = 0;
let ballY_4 = 0;
let currentYaw_4 = 0;

robot4Listener.subscribe((message) => {
    const pitch = message.pitch;
    const roll = message.roll;
    const yaw = message.yaw;

    robotX_4 = message.robot_x * 0.5;
    robotY_4 = message.robot_y * 0.5;
    ballX_4 = message.ball_x * 0.5;
    ballY_4 = message.ball_y * 0.5;

    draw4();

    document.getElementById("orientation4").innerText =
        `Roll: ${roll.toFixed(1)}°, Pitch: ${pitch.toFixed(1)}°, Yaw: ${yaw.toFixed(1)}°`;

    updateYawDial4(yaw);
    updateYawOnCanvas4(yaw + 90);
});

function updateYawDial4(yaw) {
    const pointer = document.getElementById("yaw-pointer4");
    const angle = yaw * (-1);
    pointer.style.transform = `rotate(${angle}deg)`;

    currentYaw_4 = yaw;
    draw4();
}

function sendTestMessage4(setValue) {
    const publisher = new ROSLIB.Topic({
        ros: ros,
        name: "/robot4sender",
        messageType: "humanoid_interfaces/Robot4senderMsg"
    });

    const msg = new ROSLIB.Message({ set: setValue });
    publisher.publish(msg);
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
    draw4();
}
// ============================================================

// ============================================================
// 공통 함수
// ============================================================
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

    if (img1.complete) 
    {
        canvas1.width = img1.width;
        canvas1.height = img1.height;
        draw1();
    } 
    else 
    {
        img1.onload = () => {
            canvas1.width = img1.width;
            canvas1.height = img1.height;
            draw1();
        };
    }

    if (img2.complete) 
    {
        canvas2.width = img2.width;
        canvas2.height = img2.height;
        draw2();
    }
    else 
    {
        img2.onload = () => {
            canvas2.width = img2.width;
            canvas2.height = img2.height;
            draw2();
        };
    }

    if (img3.complete) 
    {
        canvas3.width = img3.width;
        canvas3.height = img3.height;
        draw3();
    } 
    else 
    {
        img3.onload = () => {
            canvas3.width = img3.width;
            canvas3.height = img3.height;
            draw3();
        };
    }

    if (img4.complete) 
    {
        canvas4.width = img4.width;
        canvas4.height = img4.height;
        draw4();
    } 
    else 
    {
        img4.onload = () => {
            canvas4.width = img4.width;
            canvas4.height = img4.height;
            draw4();
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
}

// ============================================================