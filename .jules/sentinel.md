## 2025-12-11 - Python Eval Sandbox Escape via Attributes
**Vulnerability:** The `ExpressionFunction` class used `ast.parse` and `eval` to evaluate user input but allowed `ast.Attribute` nodes. This permitted attackers to access internal attributes (like `__class__`, `__doc__`) even with empty `__builtins__`, potentially leading to sandbox escape.
**Learning:** Restricting `__builtins__` is insufficient if the AST allows traversing object attributes. `ast.Attribute` should be disallowed unless strictly necessary and carefully validated.
**Prevention:** Whitelist specific AST nodes only. For mathematical expressions, disallow `ast.Attribute` and `ast.Call` on anything other than `ast.Name`.

## 2025-12-12 - Unsafe Eval in PyQt UI Controller
**Vulnerability:** The `PendulumController._safe_eval` method in the UI used raw `eval` with restricted globals but without AST validation, leaving it vulnerable to the same attribute traversal attacks as `ExpressionFunction` previously.
**Learning:** Security fixes in core libraries (like `ExpressionFunction`) do not automatically propagate to consumers if they reimplement the logic. Code duplication of security-critical logic is a major risk.
**Prevention:** Centralize security logic. Avoid reimplementing `safe_eval` in UI layers; reuse the hardened `ExpressionFunction` or a shared utility.
