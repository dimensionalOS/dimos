Changelog
=========

.. currentmodule:: aioice

0.7.7
-----

 * Close underlying transport when a TURN allocation is deleted.
 * Shutdown mDNS stack when it is no longer referenced.
 * Rewrite asynchronous tests as coroutines.

0.7.6
-----

 * Ensure `dnspython` version is at least 2.0.0.
 * Avoid 400 error when using TURN with concurrent sends.
 * Avoid error if a connection is lost before the local candidate is set.

0.7.5
-----

 * Emit an event when the ICE connection is closed.

0.7.4
-----

 * Perform mDNS resolution using an A query, as Firefox does not respond
   to ANY queries.
 * Use full module name to name loggers.

0.7.3
-----

 * Defer mDNS lock initialisation to avoid mismatched event-loops,
   fixes errors seen with uvloop.
 * Correctly gather STUN candidates when there are multiple components.

0.7.2
-----

 * Add support for resolving mDNS candidates.

0.7.1
-----

TURN
....

 * Use the LIFETIME attribute returned by the server to determine the
   time-to-expiry for the allocation.
 * Raise stun.TransactionFailed if TURN allocation request is rejected
   with an error.
 * Handle 438 (Stale Nonce) error responses.
 * Ignore STUN transaction errors when deleting TURN allocation.
 * Periodically refresh channel bindings.

0.7.0
-----

Breaking
........

 * Make :meth:`Connection.add_remote_candidate` a coroutine.
 * Remove the `Connection.remote_candidates` setter.
