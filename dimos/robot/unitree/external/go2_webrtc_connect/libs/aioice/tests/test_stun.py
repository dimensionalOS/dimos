import unittest
from binascii import unhexlify
from collections import OrderedDict

from aioice import stun

from .utils import asynctest, read_message


class AttributeTest(unittest.TestCase):
    def test_unpack_error_code(self):
        data = unhexlify("00000457526f6c6520436f6e666c696374")
        code, reason = stun.unpack_error_code(data)
        self.assertEqual(code, 487)
        self.assertEqual(reason, "Role Conflict")

    def test_unpack_error_code_too_short(self):
        data = unhexlify("000004")
        with self.assertRaises(ValueError) as cm:
            stun.unpack_error_code(data)
        self.assertEqual(str(cm.exception), "STUN error code is less than 4 bytes")

    def test_unpack_xor_address_ipv4(self):
        transaction_id = unhexlify("b7e7a701bc34d686fa87dfae")
        address, port = stun.unpack_xor_address(
            unhexlify("0001a147e112a643"), transaction_id
        )
        self.assertEqual(address, "192.0.2.1")
        self.assertEqual(port, 32853)

    def test_unpack_xor_address_ipv4_truncated(self):
        transaction_id = unhexlify("b7e7a701bc34d686fa87dfae")
        with self.assertRaises(ValueError) as cm:
            stun.unpack_xor_address(unhexlify("0001a147e112a6"), transaction_id)
        self.assertEqual(str(cm.exception), "STUN address has invalid length for IPv4")

    def test_unpack_xor_address_ipv6(self):
        transaction_id = unhexlify("b7e7a701bc34d686fa87dfae")
        address, port = stun.unpack_xor_address(
            unhexlify("0002a1470113a9faa5d3f179bc25f4b5bed2b9d9"), transaction_id
        )
        self.assertEqual(address, "2001:db8:1234:5678:11:2233:4455:6677")
        self.assertEqual(port, 32853)

    def test_unpack_xor_address_ipv6_truncated(self):
        transaction_id = unhexlify("b7e7a701bc34d686fa87dfae")
        with self.assertRaises(ValueError) as cm:
            stun.unpack_xor_address(
                unhexlify("0002a1470113a9faa5d3f179bc25f4b5bed2b9"), transaction_id
            )
        self.assertEqual(str(cm.exception), "STUN address has invalid length for IPv6")

    def test_unpack_xor_address_too_short(self):
        transaction_id = unhexlify("b7e7a701bc34d686fa87dfae")
        with self.assertRaises(ValueError) as cm:
            stun.unpack_xor_address(unhexlify("0001"), transaction_id)
        self.assertEqual(str(cm.exception), "STUN address length is less than 4 bytes")

    def test_unpack_xor_address_unknown_protocol(self):
        transaction_id = unhexlify("b7e7a701bc34d686fa87dfae")
        with self.assertRaises(ValueError) as cm:
            stun.unpack_xor_address(unhexlify("0003a147e112a643"), transaction_id)
        self.assertEqual(str(cm.exception), "STUN address has unknown protocol")

    def test_pack_error_code(self):
        data = stun.pack_error_code((487, "Role Conflict"))
        self.assertEqual(data, unhexlify("00000457526f6c6520436f6e666c696374"))

    def test_pack_xor_address_ipv4(self):
        transaction_id = unhexlify("b7e7a701bc34d686fa87dfae")
        data = stun.pack_xor_address(("192.0.2.1", 32853), transaction_id)
        self.assertEqual(data, unhexlify("0001a147e112a643"))

    def test_pack_xor_address_ipv6(self):
        transaction_id = unhexlify("b7e7a701bc34d686fa87dfae")
        data = stun.pack_xor_address(
            ("2001:db8:1234:5678:11:2233:4455:6677", 32853), transaction_id
        )
        self.assertEqual(data, unhexlify("0002a1470113a9faa5d3f179bc25f4b5bed2b9d9"))

    def test_pack_xor_address_unknown_protocol(self):
        transaction_id = unhexlify("b7e7a701bc34d686fa87dfae")
        with self.assertRaises(ValueError) as cm:
            stun.pack_xor_address(("foo", 32853), transaction_id)
        self.assertEqual(
            str(cm.exception), "'foo' does not appear to be an IPv4 or IPv6 address"
        )


class MessageTest(unittest.TestCase):
    def test_binding_request(self):
        data = read_message("binding_request.bin")

        message = stun.parse_message(data)
        self.assertEqual(message.message_method, stun.Method.BINDING)
        self.assertEqual(message.message_class, stun.Class.REQUEST)
        self.assertEqual(message.transaction_id, b"Nvfx3lU7FUBF")
        self.assertEqual(message.attributes, OrderedDict())

        self.assertEqual(bytes(message), data)
        self.assertEqual(
            repr(message),
            "Message(message_method=Method.BINDING, message_class=Class.REQUEST, "
            "transaction_id=b'Nvfx3lU7FUBF')",
        )

    def test_binding_request_ice_controlled(self):
        data = read_message("binding_request_ice_controlled.bin")

        message = stun.parse_message(data)
        self.assertEqual(message.message_method, stun.Method.BINDING)
        self.assertEqual(message.message_class, stun.Class.REQUEST)
        self.assertEqual(message.transaction_id, b"wxaNbAdXjwG3")
        self.assertEqual(
            message.attributes,
            OrderedDict(
                [
                    ("USERNAME", "AYeZ:sw7YvCSbcVex3bhi"),
                    ("PRIORITY", 1685987071),
                    ("SOFTWARE", "FreeSWITCH (-37-987c9b9 64bit)"),
                    ("ICE-CONTROLLED", 5491930053772927353),
                    (
                        "MESSAGE-INTEGRITY",
                        unhexlify("1963108a4f764015a66b3fea0b1883dfde1436c8"),
                    ),
                    ("FINGERPRINT", 3230414530),
                ]
            ),
        )

        self.assertEqual(bytes(message), data)

    def test_binding_request_ice_controlled_bad_fingerprint(self):
        data = read_message("binding_request_ice_controlled.bin")[0:-1] + b"z"

        with self.assertRaises(ValueError) as cm:
            stun.parse_message(data)
        self.assertEqual(str(cm.exception), "STUN message fingerprint does not match")

    def test_binding_request_ice_controlled_bad_integrity(self):
        data = read_message("binding_request_ice_controlled.bin")

        with self.assertRaises(ValueError) as cm:
            stun.parse_message(data, integrity_key=b"bogus-key")
        self.assertEqual(str(cm.exception), "STUN message integrity does not match")

    def test_binding_request_ice_controlling(self):
        data = read_message("binding_request_ice_controlling.bin")

        message = stun.parse_message(data)
        self.assertEqual(message.message_method, stun.Method.BINDING)
        self.assertEqual(message.message_class, stun.Class.REQUEST)
        self.assertEqual(message.transaction_id, b"JEwwUxjLWaa2")
        self.assertEqual(
            message.attributes,
            OrderedDict(
                [
                    ("USERNAME", "sw7YvCSbcVex3bhi:AYeZ"),
                    ("ICE-CONTROLLING", 5943294521425135761),
                    ("USE-CANDIDATE", None),
                    ("PRIORITY", 1853759231),
                    (
                        "MESSAGE-INTEGRITY",
                        unhexlify("c87b58eccbacdbc075d497ad0c965a82937ab587"),
                    ),
                    ("FINGERPRINT", 1347006354),
                ]
            ),
        )

    def test_binding_response(self):
        data = read_message("binding_response.bin")

        message = stun.parse_message(data)
        self.assertEqual(message.message_method, stun.Method.BINDING)
        self.assertEqual(message.message_class, stun.Class.RESPONSE)
        self.assertEqual(message.transaction_id, b"Nvfx3lU7FUBF")
        self.assertEqual(
            message.attributes,
            OrderedDict(
                [
                    ("XOR-MAPPED-ADDRESS", ("80.200.136.90", 53054)),
                    ("MAPPED-ADDRESS", ("80.200.136.90", 53054)),
                    ("RESPONSE-ORIGIN", ("52.17.36.97", 3478)),
                    ("OTHER-ADDRESS", ("52.17.36.97", 3479)),
                    ("SOFTWARE", "Citrix-3.2.4.5 'Marshal West'"),
                ]
            ),
        )

        self.assertEqual(bytes(message), data)

    def test_message_body_length_mismatch(self):
        data = read_message("binding_response.bin") + b"123"
        with self.assertRaises(ValueError) as cm:
            stun.parse_message(data)
        self.assertEqual(str(cm.exception), "STUN message length does not match")

    def test_message_shorter_than_header(self):
        with self.assertRaises(ValueError) as cm:
            stun.parse_message(b"123")
        self.assertEqual(str(cm.exception), "STUN message length is less than 20 bytes")

    def test_message_with_unknown_method(self):
        with self.assertRaises(ValueError) as cm:
            stun.parse_message(bytes(20))
        self.assertEqual(str(cm.exception), "0 is not a valid Method")


class TransactionTest(unittest.TestCase):
    def setUp(self):
        stun.RETRY_MAX = 0
        stun.RETRY_RTO = 0

    def tearDown(self):
        stun.RETRY_MAX = 6
        stun.RETRY_RTO = 0.5

    @asynctest
    async def test_timeout(self):
        class DummyProtocol:
            def send_stun(self, message, address):
                pass

        request = stun.Message(
            message_method=stun.Method.BINDING, message_class=stun.Class.REQUEST
        )
        transaction = stun.Transaction(request, ("127.0.0.1", 1234), DummyProtocol())

        # timeout
        with self.assertRaises(stun.TransactionTimeout):
            await transaction.run()

        # receive response after timeout
        response = stun.Message(
            message_method=stun.Method.BINDING, message_class=stun.Class.RESPONSE
        )
        transaction.response_received(response, ("127.0.0.1", 1234))
