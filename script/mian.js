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

// ROS 토픽 구독 설정
const robot1Listener = new ROSLIB.Topic({
    ros,
    name: "/robot1receiver",
    messageType: "humanoid_interfaces/Robot1receiverMsg",
});

let robotX = 275;
let robotY = 200;
let ballX = 0;
let ballY = 0;
let currentYaw = 0;

robot1Listener.subscribe((message) => {
    const pitch = message.pitch;
    const roll = message.roll;
    const yaw = message.yaw;

    robotX = message.robot_x * 0.5;
    robotY = message.robot_y * 0.5;
    ballX = message.ball_x * 0.5;
    ballY = message.ball_y * 0.5;

    draw();

    document.getElementById("orientation").innerText =
        `Roll: ${roll.toFixed(1)}°, Pitch: ${pitch.toFixed(1)}°, Yaw: ${yaw.toFixed(1)}°`;
    
    updateYawDial(yaw);
    updateYawOnCanvas(yaw + 90);
});

function updateYawDial(yaw) {
    const pointer = document.getElementById("yaw-pointer");
    const angle = yaw * (-1);
    pointer.style.transform = `rotate(${angle}deg)`;

    currentYaw = yaw;
    draw();
}

function sendTestMessage(setValue) {
    const publisher = new ROSLIB.Topic({
        ros: ros,
        name: "/robot1sender",
        messageType: "humanoid_interfaces/Robot1senderMsg"
    });

    const msg = new ROSLIB.Message({ set: setValue });
    publisher.publish(msg);
}

const arrowLength = 15;

// 캔버스 관련 변수
let ctx, img, canvas;

function draw() {
    if (!ctx || !img) return;

    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.drawImage(img, 0, 0, canvas.width, canvas.height);

    // 로봇 위치 (파란 원)
    ctx.fillStyle = "blue";
    ctx.beginPath();
    ctx.arc(robotX, robotY, 10, 0, Math.PI * 2);
    ctx.fill();

    // 방향선 (yaw)
    const yawRad = currentYaw * Math.PI / 180;
    const endX = robotX + arrowLength * Math.cos(yawRad);
    const endY = robotY - arrowLength * Math.sin(yawRad);

    ctx.strokeStyle = "black";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(robotX, robotY);
    ctx.lineTo(endX, endY);
    ctx.stroke();

    // 공 위치 (빨간 원)
    ctx.fillStyle = "red";
    ctx.beginPath();
    ctx.arc(ballX, ballY, 4, 0, Math.PI * 2);
    ctx.fill();
}

// 외부에서 yaw 업데이트 시 draw() 호출
function updateYawOnCanvas(yawDeg) {
    currentYaw = yawDeg;
    draw();
}

window.onload = function () {
    // 탭 초기화
    const defaultTab = document.querySelector('.tab[href="#"][onclick*="robot1"]');
    showTab('robot1', defaultTab);

    // 캔버스 및 이미지 초기화
    img = document.getElementById("fieldImage1");
    canvas = document.getElementById("fieldCanvas1");
    ctx = canvas.getContext("2d");

    if (img.complete) {
        canvas.width = img.width;
        canvas.height = img.height;
        draw();
    } else {
        img.onload = () => {
            canvas.width = img.width;
            canvas.height = img.height;
            draw();
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
