/**
 * 通用 DOM 元素拖拽增强工具
 * @param {HTMLElement} panel 要移动的容器元素
 * @param {HTMLElement} handle 拖拽把手（默认使用 panel 自身）
 */
export function makeDraggable(panel, handle = null) {
    if (!panel) return;
    const dragHandle = handle || panel;
    dragHandle.style.cursor = 'grab';
    dragHandle.style.userSelect = 'none';

    let isDragging = false;
    let startX = 0, startY = 0;
    let initialLeft = 0, initialTop = 0;

    dragHandle.addEventListener('mousedown', (e) => {
        // 忽略按钮、输入框、下拉框、文本链接等交互控件的点击
        if (['BUTTON', 'INPUT', 'SELECT', 'TEXTAREA', 'A'].includes(e.target.tagName)) {
            return;
        }

        isDragging = true;
        dragHandle.style.cursor = 'grabbing';
        startX = e.clientX;
        startY = e.clientY;

        const rect = panel.getBoundingClientRect();
        // 清除居中用的 transform 以免干扰绝对定位计算
        panel.style.transform = 'none';
        panel.style.left = `${rect.left}px`;
        panel.style.top = `${rect.top}px`;
        panel.style.right = 'auto';
        panel.style.bottom = 'auto';
        panel.style.margin = '0';

        initialLeft = rect.left;
        initialTop = rect.top;

        e.preventDefault();
    });

    window.addEventListener('mousemove', (e) => {
        if (!isDragging) return;
        const dx = e.clientX - startX;
        const dy = e.clientY - startY;

        let newLeft = initialLeft + dx;
        let newTop = initialTop + dy;

        // 视口边界保护
        newLeft = Math.max(0, Math.min(window.innerWidth - panel.offsetWidth, newLeft));
        newTop = Math.max(0, Math.min(window.innerHeight - panel.offsetHeight, newTop));

        panel.style.left = `${newLeft}px`;
        panel.style.top = `${newTop}px`;
    });

    window.addEventListener('mouseup', () => {
        if (isDragging) {
            isDragging = false;
            dragHandle.style.cursor = 'grab';
        }
    });
}
