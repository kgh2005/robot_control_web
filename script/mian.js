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

    if (img1.complete) {
        canvas1.width = img1.width;
        canvas1.height = img1.height;
        draw1();
    } else {
        img1.onload = () => {
            canvas1.width = img1.width;
            canvas1.height = img1.height;
            draw1();
        };
    }

    if (img2.complete) {
        canvas2.width = img2.width;
        canvas2.height = img2.height;
        draw2();
    } else {
        img2.onload = () => {
            canvas2.width = img2.width;
            canvas2.height = img2.height;
            draw2();
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