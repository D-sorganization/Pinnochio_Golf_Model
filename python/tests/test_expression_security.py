import pytest
from double_pendulum_model.physics.double_pendulum import ExpressionFunction, DoublePendulumState


def test_expression_no_attributes() -> None:
    """Ensure accessing attributes is forbidden to prevent sandbox escape."""
    # Example expression: (1).__class__
    with pytest.raises(ValueError, match="Disallowed syntax in expression: Attribute"):
        ExpressionFunction("(1).__class__")


def test_call_on_attribute_blocked() -> None:
    """Ensure calling methods on objects is forbidden."""
    # Example expression: theta1.as_integer_ratio()
    # This parses as a Call node where func is an Attribute node.
    # The validation logic checks the Call node and rejects it because func is not a Name.
    with pytest.raises(ValueError, match="Only direct function calls are permitted"):
        ExpressionFunction("theta1.as_integer_ratio()")


def test_expression_disallowed_function() -> None:
    """Ensure calling disallowed functions is forbidden."""
    # Example expression: eval('1')
    with pytest.raises(ValueError, match="Function 'eval' is not permitted"):
        ExpressionFunction("eval('1')")


def test_expression_no_list_comp() -> None:
    """Ensure list comprehensions are forbidden."""
    # List comprehensions (e.g., [x for x in range(10)]) are not in the allowed nodes.
    with pytest.raises(ValueError, match="Disallowed syntax in expression: ListComp"):
        ExpressionFunction("[x for x in range(10)]")


def test_expression_no_dict() -> None:
    """Ensure dictionaries are forbidden."""
    # Dictionaries (e.g., {'a': 1}) are not in the allowed nodes.
    with pytest.raises(ValueError, match="Disallowed syntax in expression: Dict"):
        ExpressionFunction("{'a': 1}")


def test_expression_valid_math() -> None:
    """Ensure valid math expressions still work."""
    expr = ExpressionFunction("sin(t) + cos(theta1) * 0.5")
    state = DoublePendulumState(theta1=0.0, theta2=0.0, omega1=0.0, omega2=0.0)
    result = expr(0.0, state)
    assert isinstance(result, float)


def test_expression_unknown_variable() -> None:
    """Ensure accessing unknown variables is forbidden."""
    with pytest.raises(ValueError, match="Use of unknown variable '__builtins__'"):
        ExpressionFunction("__builtins__")
