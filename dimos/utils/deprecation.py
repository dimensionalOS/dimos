import warnings
import functools

def deprecated(reason: str):
    """
    This function itself is deprecated as we can use `from warnings import deprecated` in Python 3.13+.
    """
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            warnings.warn(
                f"{func.__name__} is deprecated: {reason}",
                category=DeprecationWarning,
                stacklevel=2
            )
            return func(*args, **kwargs)
        return wrapper
    return decorator