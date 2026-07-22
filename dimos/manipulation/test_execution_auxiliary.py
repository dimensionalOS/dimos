from dimos.manipulation.execution_auxiliary import AuxiliaryCallBook
from dimos.manipulation.execution_effects import AuxiliaryDone


def test_expire_retains_timeout_result_through_late_completion() -> None:
    book = AuxiliaryCallBook()
    ticket = book.register("aux", 1.0, setter=True)

    assert book.expire(2.0, "timeout")
    assert book.has_inflight()
    assert book.complete(AuxiliaryDone(ticket.action_id, "late value"))
    assert not book.has_unsettled()
    assert book.take_result(ticket.action_id) == (False, None, "timeout")
