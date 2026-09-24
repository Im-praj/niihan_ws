import re

with open('src/niihan_dashboard/web/js/dashboard.js', 'r') as f:
    text = f.read()

old_joy = """joyKnob.addEventListener('mousedown', () => { isDraggingJoy = true; });
document.addEventListener('mouseup', () => {
    if (isDraggingJoy) {
        isDraggingJoy = false;
        joyKnob.style.top = '50px';
        joyKnob.style.left = '50px';
        sendJoyCmd(0, 0);
    }
});
document.addEventListener('mousemove', (e) => {
    if (!isDraggingJoy) return;
    
    const rect = joyZone.getBoundingClientRect();
    const centerX = rect.left + 75;
    const centerY = rect.top + 75;
    
    let dx = e.clientX - centerX;
    let dy = e.clientY - centerY;"""

new_joy = """// Pointer events for mobile + desktop support
joyZone.style.touchAction = 'none'; // Prevent scrolling

joyKnob.addEventListener('pointerdown', (e) => { 
    isDraggingJoy = true; 
    joyKnob.setPointerCapture(e.pointerId);
});

joyKnob.addEventListener('pointerup', (e) => {
    if (isDraggingJoy) {
        isDraggingJoy = false;
        joyKnob.releasePointerCapture(e.pointerId);
        joyKnob.style.top = '50px';
        joyKnob.style.left = '50px';
        sendJoyCmd(0, 0);
    }
});
joyKnob.addEventListener('pointercancel', (e) => {
    if (isDraggingJoy) {
        isDraggingJoy = false;
        joyKnob.releasePointerCapture(e.pointerId);
        joyKnob.style.top = '50px';
        joyKnob.style.left = '50px';
        sendJoyCmd(0, 0);
    }
});

joyKnob.addEventListener('pointermove', (e) => {
    if (!isDraggingJoy) return;
    
    const rect = joyZone.getBoundingClientRect();
    // Assuming joyZone is 150x150, center is 75,75 relative to joyZone
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;
    
    let dx = e.clientX - centerX;
    let dy = e.clientY - centerY;"""

text = text.replace(old_joy, new_joy)

with open('src/niihan_dashboard/web/js/dashboard.js', 'w') as f:
    f.write(text)
