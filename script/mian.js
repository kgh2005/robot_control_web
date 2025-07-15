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


// IMU 토픽 구독
const imuListener = new ROSLIB.Topic({
    ros,
    name: "/Imu",
    messageType: "humanoid_interfaces/ImuMsg",
});

imuListener.subscribe((message) => {
    const pitch = message.pitch;
    const roll = message.roll;
    const yaw = message.yaw;

    // HTML 요소에 표시
    document.getElementById("orientation").innerText =
        `Roll: ${roll.toFixed(1)}°, Pitch: ${pitch.toFixed(1)}°, Yaw: ${yaw.toFixed(1)}°`;

    updateYawDial(yaw);
});

function updateYawDial(yaw) {
    const pointer = document.getElementById("yaw-pointer");

    const angle = yaw * (-1);

    pointer.style.transform = `rotate(${angle}deg)`;
}

function sendTestMessage(setValue) {
    console.log("버튼이 눌렸습니다! Set =", setValue);

    const publisher = new ROSLIB.Topic({
        ros: ros,
        name: "/Imuflag",
        messageType: "humanoid_interfaces/ImuflagMsg"
    });

    const msg = new ROSLIB.Message({
        set: setValue
    });

    publisher.publish(msg);
}

window.onload = function () {
    const img = document.getElementById("fieldImage");
    const canvas = document.getElementById("fieldCanvas");
    const ctx = canvas.getContext("2d");

    function drawCenterDot() {
        canvas.width = img.width;
        canvas.height = img.height;

        const centerX = 275;
        const centerY = 200;

        ctx.fillStyle = "blue";
        ctx.beginPath();
        ctx.arc(centerX, centerY, 10, 0, Math.PI * 2);
        ctx.fill();
    }

    if (img.complete) {
        drawCenterDot(); // 이미지 이미 로드된 경우 바로 실행
    } else {
        img.onload = drawCenterDot; // 아니라면 onload에 등록
    }
};

