const socket = new WebSocket('ws://localhost:8080/video');
const img = document.getElementById('robot-view');

socket.onmessage = (event) => {
    // Assuming Go sends raw JPEG blob
    const blob = new Blob([event.data], { type: 'image/jpeg' });
    img.src = URL.createObjectURL(blob);
};
