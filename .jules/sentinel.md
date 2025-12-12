## 2025-12-11 - Python Eval Sandbox Escape via Attributes
**Vulnerability:** The `ExpressionFunction` class used `ast.parse` and `eval` to evaluate user input but allowed `ast.Attribute` nodes. This permitted attackers to access internal attributes (like `__class__`, `__doc__`) even with empty `__builtins__`, potentially leading to sandbox escape.
**Learning:** Restricting `__builtins__` is insufficient if the AST allows traversing object attributes. `ast.Attribute` should be disallowed unless strictly necessary and carefully validated.
**Prevention:** Whitelist specific AST nodes only. For mathematical expressions, disallow `ast.Attribute` and `ast.Call` on anything other than `ast.Name`.
