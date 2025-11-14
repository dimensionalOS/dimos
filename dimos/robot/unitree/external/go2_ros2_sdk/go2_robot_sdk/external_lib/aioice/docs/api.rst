API Reference
=============

.. automodule:: aioice

   .. autoclass:: Connection
      :members: local_candidates, local_username, local_password, remote_candidates, remote_username, remote_password

      .. automethod:: add_remote_candidate
      .. automethod:: gather_candidates
      .. automethod:: get_default_candidate
      .. automethod:: connect
      .. automethod:: close
      .. automethod:: recv
      .. automethod:: recvfrom
      .. automethod:: send
      .. automethod:: sendto
      .. automethod:: set_selected_pair

   .. autoclass:: Candidate

      .. automethod:: from_sdp
      .. automethod:: to_sdp
