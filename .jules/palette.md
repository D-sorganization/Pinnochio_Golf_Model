# Palette's Journal

## 2025-12-01 - Quantitative Transparency
**Learning:** Users trust tools more when they can see the numbers driving the visualization (e.g., specific joint angles). Visual-only feedback is insufficient for precise engineering tasks.
**Action:** Always provide numerical readouts alongside analog controls (sliders/dials).

## 2025-12-05 - Immediate Input Validation
**Learning:** For complex inputs like mathematical expressions, silent failure (returning 0.0) causes confusion. Immediate visual feedback (red background/tooltips) during typing prevents errors before execution.
**Action:** Validate text inputs on `textChanged` and provide inline visual cues for syntax errors.
