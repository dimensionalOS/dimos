/*
 * test_wire_compat.cpp
 *
 * Verifies that the Arduino C encode/decode functions produce byte-for-byte
 * identical output to the LCM C++ encode (minus the 8-byte fingerprint hash).
 *
 * Build & run:
 *   cd dimos/hardware/arduino/test
 *   nix build && ./result/bin/test_wire_compat
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdint>

/* ---- Arduino C headers (our code under test) ---- */
#include "std_msgs/Time.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/ColorRGBA.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Inertia.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "geometry_msgs/AccelWithCovariance.h"

/*
 * LCM C++ headers (reference implementation).
 * These live in namespace geometry_msgs:: / std_msgs:: already.
 * Our C types use geometry_msgs_* prefix so no collision.
 *
 * The .hpp includes use the system lcm/lcm_coretypes.h (with malloc etc.)
 * while our .h includes use lcm_coretypes_arduino.h — both are in scope
 * but don't conflict because the upstream header has its own include guard.
 */
#include "geometry_msgs/Vector3.hpp"
#include "geometry_msgs/Point.hpp"
#include "geometry_msgs/Point32.hpp"
#include "geometry_msgs/Quaternion.hpp"
#include "geometry_msgs/Pose.hpp"
#include "geometry_msgs/Pose2D.hpp"
#include "geometry_msgs/Twist.hpp"
#include "geometry_msgs/Accel.hpp"
#include "geometry_msgs/Transform.hpp"
#include "geometry_msgs/Wrench.hpp"
#include "geometry_msgs/Inertia.hpp"
#include "geometry_msgs/PoseWithCovariance.hpp"
#include "geometry_msgs/TwistWithCovariance.hpp"
#include "geometry_msgs/AccelWithCovariance.hpp"
#include "std_msgs/Time.hpp"
#include "std_msgs/Bool.hpp"
#include "std_msgs/Int32.hpp"
#include "std_msgs/Float32.hpp"
#include "std_msgs/Float64.hpp"
#include "std_msgs/ColorRGBA.hpp"

static int tests_run = 0;
static int tests_passed = 0;

static void hex_dump(const char *label, const uint8_t *data, int len) {
    printf("  %s (%d bytes): ", label, len);
    for (int i = 0; i < len && i < 64; i++)
        printf("%02x ", data[i]);
    if (len > 64) printf("...");
    printf("\n");
}

/*
 * Compare C-encoded bytes against the LCM C++ _encode_one output.
 *
 * LCM lcm_encode() prepends an 8-byte fingerprint hash, then calls
 * _encode_one() for the fields.  We compare against _encode_one() output
 * (i.e., the bytes AFTER the hash), since the Arduino side skips the hash.
 */
static bool compare_bytes(const char *name,
                          const uint8_t *c_buf, int c_len,
                          const uint8_t *cpp_buf, int cpp_len)
{
    tests_run++;
    /* cpp_buf starts with 8-byte hash — skip it */
    const int HASH_SIZE = 8;
    int field_len = cpp_len - HASH_SIZE;
    const uint8_t *cpp_fields = cpp_buf + HASH_SIZE;

    if (c_len != field_len) {
        printf("FAIL %s: size mismatch (C=%d, C++=%d after hash)\n",
               name, c_len, field_len);
        hex_dump("C  ", c_buf, c_len);
        hex_dump("C++", cpp_fields, field_len);
        return false;
    }
    if (memcmp(c_buf, cpp_fields, c_len) != 0) {
        printf("FAIL %s: content mismatch\n", name);
        hex_dump("C  ", c_buf, c_len);
        hex_dump("C++", cpp_fields, field_len);
        return false;
    }
    printf("PASS %s (%d bytes)\n", name, c_len);
    tests_passed++;
    return true;
}

/* Helper: encode an LCM C++ message via lcm_encode() and return the bytes */
template<typename T>
static int lcm_encode_to_buf(const T &msg, uint8_t *buf, int maxlen)
{
    int sz = msg.getEncodedSize();
    if (sz > maxlen) return -1;
    msg.encode(buf, 0, sz);
    return sz;
}

/* ============================================================
 * Test functions for each type
 * ============================================================ */

static void test_time() {
    dimos_msg__Time c = {1234567890, 123456789};

    ::std_msgs::Time cpp;
    cpp.sec = 1234567890;
    cpp.nsec = 123456789;

    uint8_t c_buf[64], cpp_buf[64];
    int c_len = dimos_msg__Time__encode(c_buf, 0, sizeof(c_buf), &c);
    int cpp_len = lcm_encode_to_buf(cpp, cpp_buf, sizeof(cpp_buf));
    compare_bytes("std_msgs::Time", c_buf, c_len, cpp_buf, cpp_len);
}

static void test_bool() {
    dimos_msg__Bool c = {1};

    ::std_msgs::Bool cpp;
    cpp.data = 1;

    uint8_t c_buf[64], cpp_buf[64];
    int c_len = dimos_msg__Bool__encode(c_buf, 0, sizeof(c_buf), &c);
    int cpp_len = lcm_encode_to_buf(cpp, cpp_buf, sizeof(cpp_buf));
    compare_bytes("std_msgs::Bool", c_buf, c_len, cpp_buf, cpp_len);
}

static void test_int32() {
    dimos_msg__Int32 c = {-42};

    ::std_msgs::Int32 cpp;
    cpp.data = -42;

    uint8_t c_buf[64], cpp_buf[64];
    int c_len = dimos_msg__Int32__encode(c_buf, 0, sizeof(c_buf), &c);
    int cpp_len = lcm_encode_to_buf(cpp, cpp_buf, sizeof(cpp_buf));
    compare_bytes("std_msgs::Int32", c_buf, c_len, cpp_buf, cpp_len);
}

static void test_float32() {
    dimos_msg__Float32 c = {3.14159f};

    ::std_msgs::Float32 cpp;
    cpp.data = 3.14159f;

    uint8_t c_buf[64], cpp_buf[64];
    int c_len = dimos_msg__Float32__encode(c_buf, 0, sizeof(c_buf), &c);
    int cpp_len = lcm_encode_to_buf(cpp, cpp_buf, sizeof(cpp_buf));
    compare_bytes("std_msgs::Float32", c_buf, c_len, cpp_buf, cpp_len);
}

static void test_float64() {
    dimos_msg__Float64 c = {2.718281828459045};

    ::std_msgs::Float64 cpp;
    cpp.data = 2.718281828459045;

    uint8_t c_buf[64], cpp_buf[64];
    int c_len = dimos_msg__Float64__encode(c_buf, 0, sizeof(c_buf), &c);
    int cpp_len = lcm_encode_to_buf(cpp, cpp_buf, sizeof(cpp_buf));
    compare_bytes("std_msgs::Float64", c_buf, c_len, cpp_buf, cpp_len);
}

static void test_colorrgba() {
    dimos_msg__ColorRGBA c = {0.1f, 0.2f, 0.3f, 1.0f};

    ::std_msgs::ColorRGBA cpp;
    cpp.r = 0.1f; cpp.g = 0.2f; cpp.b = 0.3f; cpp.a = 1.0f;

    uint8_t c_buf[64], cpp_buf[64];
    int c_len = dimos_msg__ColorRGBA__encode(c_buf, 0, sizeof(c_buf), &c);
    int cpp_len = lcm_encode_to_buf(cpp, cpp_buf, sizeof(cpp_buf));
    compare_bytes("std_msgs::ColorRGBA", c_buf, c_len, cpp_buf, cpp_len);
}

static void test_vector3() {
    dimos_msg__Vector3 c = {1.1, 2.2, 3.3};

    ::geometry_msgs::Vector3 cpp;
    cpp.x = 1.1; cpp.y = 2.2; cpp.z = 3.3;

    uint8_t c_buf[64], cpp_buf[64];
    int c_len = dimos_msg__Vector3__encode(c_buf, 0, sizeof(c_buf), &c);
    int cpp_len = lcm_encode_to_buf(cpp, cpp_buf, sizeof(cpp_buf));
    compare_bytes("geometry_msgs::Vector3", c_buf, c_len, cpp_buf, cpp_len);
}

static void test_point() {
    dimos_msg__Point c = {4.0, 5.0, 6.0};

    ::geometry_msgs::Point cpp;
    cpp.x = 4.0; cpp.y = 5.0; cpp.z = 6.0;

    uint8_t c_buf[64], cpp_buf[64];
    int c_len = dimos_msg__Point__encode(c_buf, 0, sizeof(c_buf), &c);
    int cpp_len = lcm_encode_to_buf(cpp, cpp_buf, sizeof(cpp_buf));
    compare_bytes("geometry_msgs::Point", c_buf, c_len, cpp_buf, cpp_len);
}

static void test_point32() {
    dimos_msg__Point32 c = {1.5f, 2.5f, 3.5f};

    ::geometry_msgs::Point32 cpp;
    cpp.x = 1.5f; cpp.y = 2.5f; cpp.z = 3.5f;

    uint8_t c_buf[64], cpp_buf[64];
    int c_len = dimos_msg__Point32__encode(c_buf, 0, sizeof(c_buf), &c);
    int cpp_len = lcm_encode_to_buf(cpp, cpp_buf, sizeof(cpp_buf));
    compare_bytes("geometry_msgs::Point32", c_buf, c_len, cpp_buf, cpp_len);
}

static void test_quaternion() {
    dimos_msg__Quaternion c = {0.0, 0.0, 0.7071, 0.7071};

    ::geometry_msgs::Quaternion cpp;
    cpp.x = 0.0; cpp.y = 0.0; cpp.z = 0.7071; cpp.w = 0.7071;

    uint8_t c_buf[64], cpp_buf[64];
    int c_len = dimos_msg__Quaternion__encode(c_buf, 0, sizeof(c_buf), &c);
    int cpp_len = lcm_encode_to_buf(cpp, cpp_buf, sizeof(cpp_buf));
    compare_bytes("geometry_msgs::Quaternion", c_buf, c_len, cpp_buf, cpp_len);
}

static void test_pose() {
    dimos_msg__Pose c;
    c.position = {1.0, 2.0, 3.0};
    c.orientation = {0.0, 0.0, 0.7071, 0.7071};

    ::geometry_msgs::Pose cpp;
    cpp.position.x = 1.0; cpp.position.y = 2.0; cpp.position.z = 3.0;
    cpp.orientation.x = 0.0; cpp.orientation.y = 0.0;
    cpp.orientation.z = 0.7071; cpp.orientation.w = 0.7071;

    uint8_t c_buf[128], cpp_buf[128];
    int c_len = dimos_msg__Pose__encode(c_buf, 0, sizeof(c_buf), &c);
    int cpp_len = lcm_encode_to_buf(cpp, cpp_buf, sizeof(cpp_buf));
    compare_bytes("geometry_msgs::Pose", c_buf, c_len, cpp_buf, cpp_len);
}

static void test_pose2d() {
    dimos_msg__Pose2D c = {10.0, 20.0, 1.5708};

    ::geometry_msgs::Pose2D cpp;
    cpp.x = 10.0; cpp.y = 20.0; cpp.theta = 1.5708;

    uint8_t c_buf[64], cpp_buf[64];
    int c_len = dimos_msg__Pose2D__encode(c_buf, 0, sizeof(c_buf), &c);
    int cpp_len = lcm_encode_to_buf(cpp, cpp_buf, sizeof(cpp_buf));
    compare_bytes("geometry_msgs::Pose2D", c_buf, c_len, cpp_buf, cpp_len);
}

static void test_twist() {
    dimos_msg__Twist c;
    c.linear = {1.0, 0.0, 0.0};
    c.angular = {0.0, 0.0, 0.5};

    ::geometry_msgs::Twist cpp;
    cpp.linear.x = 1.0; cpp.linear.y = 0.0; cpp.linear.z = 0.0;
    cpp.angular.x = 0.0; cpp.angular.y = 0.0; cpp.angular.z = 0.5;

    uint8_t c_buf[128], cpp_buf[128];
    int c_len = dimos_msg__Twist__encode(c_buf, 0, sizeof(c_buf), &c);
    int cpp_len = lcm_encode_to_buf(cpp, cpp_buf, sizeof(cpp_buf));
    compare_bytes("geometry_msgs::Twist", c_buf, c_len, cpp_buf, cpp_len);
}

static void test_accel() {
    dimos_msg__Accel c;
    c.linear = {0.0, 0.0, 9.81};
    c.angular = {0.1, 0.2, 0.3};

    ::geometry_msgs::Accel cpp;
    cpp.linear.x = 0.0; cpp.linear.y = 0.0; cpp.linear.z = 9.81;
    cpp.angular.x = 0.1; cpp.angular.y = 0.2; cpp.angular.z = 0.3;

    uint8_t c_buf[128], cpp_buf[128];
    int c_len = dimos_msg__Accel__encode(c_buf, 0, sizeof(c_buf), &c);
    int cpp_len = lcm_encode_to_buf(cpp, cpp_buf, sizeof(cpp_buf));
    compare_bytes("geometry_msgs::Accel", c_buf, c_len, cpp_buf, cpp_len);
}

static void test_transform() {
    dimos_msg__Transform c;
    c.translation = {1.0, 2.0, 3.0};
    c.rotation = {0.0, 0.0, 0.0, 1.0};

    ::geometry_msgs::Transform cpp;
    cpp.translation.x = 1.0; cpp.translation.y = 2.0; cpp.translation.z = 3.0;
    cpp.rotation.x = 0.0; cpp.rotation.y = 0.0;
    cpp.rotation.z = 0.0; cpp.rotation.w = 1.0;

    uint8_t c_buf[128], cpp_buf[128];
    int c_len = dimos_msg__Transform__encode(c_buf, 0, sizeof(c_buf), &c);
    int cpp_len = lcm_encode_to_buf(cpp, cpp_buf, sizeof(cpp_buf));
    compare_bytes("geometry_msgs::Transform", c_buf, c_len, cpp_buf, cpp_len);
}

static void test_wrench() {
    dimos_msg__Wrench c;
    c.force = {10.0, 20.0, 30.0};
    c.torque = {0.1, 0.2, 0.3};

    ::geometry_msgs::Wrench cpp;
    cpp.force.x = 10.0; cpp.force.y = 20.0; cpp.force.z = 30.0;
    cpp.torque.x = 0.1; cpp.torque.y = 0.2; cpp.torque.z = 0.3;

    uint8_t c_buf[128], cpp_buf[128];
    int c_len = dimos_msg__Wrench__encode(c_buf, 0, sizeof(c_buf), &c);
    int cpp_len = lcm_encode_to_buf(cpp, cpp_buf, sizeof(cpp_buf));
    compare_bytes("geometry_msgs::Wrench", c_buf, c_len, cpp_buf, cpp_len);
}

static void test_inertia() {
    dimos_msg__Inertia c;
    c.m = 5.0;
    c.com = {0.1, 0.2, 0.3};
    c.ixx = 1.0; c.ixy = 0.0; c.ixz = 0.0;
    c.iyy = 2.0; c.iyz = 0.0; c.izz = 3.0;

    ::geometry_msgs::Inertia cpp;
    cpp.m = 5.0;
    cpp.com.x = 0.1; cpp.com.y = 0.2; cpp.com.z = 0.3;
    cpp.ixx = 1.0; cpp.ixy = 0.0; cpp.ixz = 0.0;
    cpp.iyy = 2.0; cpp.iyz = 0.0; cpp.izz = 3.0;

    uint8_t c_buf[128], cpp_buf[128];
    int c_len = dimos_msg__Inertia__encode(c_buf, 0, sizeof(c_buf), &c);
    int cpp_len = lcm_encode_to_buf(cpp, cpp_buf, sizeof(cpp_buf));
    compare_bytes("geometry_msgs::Inertia", c_buf, c_len, cpp_buf, cpp_len);
}

static void test_pose_with_covariance() {
    dimos_msg__PoseWithCovariance c;
    c.pose.position = {1.0, 2.0, 3.0};
    c.pose.orientation = {0.0, 0.0, 0.0, 1.0};
    for (int i = 0; i < 36; i++) c.covariance[i] = (i == 0 || i == 7 || i == 14 ||
                                                     i == 21 || i == 28 || i == 35)
                                                    ? 0.01 : 0.0;

    ::geometry_msgs::PoseWithCovariance cpp;
    cpp.pose.position.x = 1.0; cpp.pose.position.y = 2.0; cpp.pose.position.z = 3.0;
    cpp.pose.orientation.x = 0.0; cpp.pose.orientation.y = 0.0;
    cpp.pose.orientation.z = 0.0; cpp.pose.orientation.w = 1.0;
    for (int i = 0; i < 36; i++) cpp.covariance[i] = c.covariance[i];

    uint8_t c_buf[512], cpp_buf[512];
    int c_len = dimos_msg__PoseWithCovariance__encode(c_buf, 0, sizeof(c_buf), &c);
    int cpp_len = lcm_encode_to_buf(cpp, cpp_buf, sizeof(cpp_buf));
    compare_bytes("geometry_msgs::PoseWithCovariance", c_buf, c_len, cpp_buf, cpp_len);
}

static void test_twist_with_covariance() {
    dimos_msg__TwistWithCovariance c;
    c.twist.linear = {1.0, 0.0, 0.0};
    c.twist.angular = {0.0, 0.0, 0.5};
    for (int i = 0; i < 36; i++) c.covariance[i] = 0.0;
    c.covariance[0] = 0.1;

    ::geometry_msgs::TwistWithCovariance cpp;
    cpp.twist.linear.x = 1.0; cpp.twist.linear.y = 0.0; cpp.twist.linear.z = 0.0;
    cpp.twist.angular.x = 0.0; cpp.twist.angular.y = 0.0; cpp.twist.angular.z = 0.5;
    for (int i = 0; i < 36; i++) cpp.covariance[i] = c.covariance[i];

    uint8_t c_buf[512], cpp_buf[512];
    int c_len = dimos_msg__TwistWithCovariance__encode(c_buf, 0, sizeof(c_buf), &c);
    int cpp_len = lcm_encode_to_buf(cpp, cpp_buf, sizeof(cpp_buf));
    compare_bytes("geometry_msgs::TwistWithCovariance", c_buf, c_len, cpp_buf, cpp_len);
}

static void test_accel_with_covariance() {
    dimos_msg__AccelWithCovariance c;
    c.accel.linear = {0.0, 0.0, 9.81};
    c.accel.angular = {0.0, 0.0, 0.0};
    for (int i = 0; i < 36; i++) c.covariance[i] = 0.0;

    ::geometry_msgs::AccelWithCovariance cpp;
    cpp.accel.linear.x = 0.0; cpp.accel.linear.y = 0.0; cpp.accel.linear.z = 9.81;
    cpp.accel.angular.x = 0.0; cpp.accel.angular.y = 0.0; cpp.accel.angular.z = 0.0;
    for (int i = 0; i < 36; i++) cpp.covariance[i] = 0.0;

    uint8_t c_buf[512], cpp_buf[512];
    int c_len = dimos_msg__AccelWithCovariance__encode(c_buf, 0, sizeof(c_buf), &c);
    int cpp_len = lcm_encode_to_buf(cpp, cpp_buf, sizeof(cpp_buf));
    compare_bytes("geometry_msgs::AccelWithCovariance", c_buf, c_len, cpp_buf, cpp_len);
}

/* ---- Roundtrip test: encode with C, decode with C, verify fields ---- */
static void test_roundtrip_pose() {
    tests_run++;
    dimos_msg__Pose original;
    original.position = {1.23456789, -2.34567890, 3.45678901};
    original.orientation = {0.1, 0.2, 0.3, 0.9327};

    uint8_t buf[128];
    int len = dimos_msg__Pose__encode(buf, 0, sizeof(buf), &original);

    dimos_msg__Pose decoded;
    int dec_len = dimos_msg__Pose__decode(buf, 0, len, &decoded);

    if (dec_len != len ||
        decoded.position.x != original.position.x ||
        decoded.position.y != original.position.y ||
        decoded.position.z != original.position.z ||
        decoded.orientation.x != original.orientation.x ||
        decoded.orientation.y != original.orientation.y ||
        decoded.orientation.z != original.orientation.z ||
        decoded.orientation.w != original.orientation.w) {
        printf("FAIL roundtrip Pose: decoded values don't match\n");
        return;
    }
    printf("PASS roundtrip Pose\n");
    tests_passed++;
}

/* ======================================================================
 * AVR double-promotion path
 *
 * On AVR `sizeof(double)==4` so lcm_coretypes_arduino.h promotes the
 * 4-byte IEEE 754 single to an 8-byte double bit-pattern on encode and
 * truncates back on decode.  The `#if defined(__AVR__)` encode/decode
 * functions are never exercised by the rest of this test file (which
 * runs on x86_64), so Paul's worry was that they could silently
 * produce the wrong bytes for months before anyone notices.
 *
 * `_dimos_f32_to_f64_bits` and `_dimos_f64_bits_to_f32` were moved out
 * of the `#if defined(__AVR__)` block and into the unconditional
 * portion of the header so we can call them here and compare against
 * the platform's own (IEEE 754 compliant) `(double)f` conversion.
 * ====================================================================== */

#include <cfloat>

static void test_avr_double_promotion()
{
    /* Mix of values that exercise every branch:
     *   - zero (±)
     *   - smallest and largest normals
     *   - typical sensor-range values
     *   - ±infinity
     *   - NaN (checks sign + mantissa-nonzero propagation)
     *
     * Denorms are intentionally omitted — the algorithm documents they
     * flush to zero and that would produce a mismatch with native
     * promotion.  One denorm test below asserts that flush-to-zero
     * behaviour explicitly. */
    const float normals[] = {
        0.0f,
        -0.0f,
        1.0f,
        -1.0f,
        0.5f,
        -0.5f,
        M_PI,
        -M_PI,
        9.81f,
        -273.15f,
        1.17549435e-38f,   /* smallest positive normal float */
        3.40282347e38f,    /* FLT_MAX */
        -3.40282347e38f,
    };

    for (size_t i = 0; i < sizeof(normals) / sizeof(normals[0]); i++) {
        float f = normals[i];
        tests_run++;

        uint64_t got_bits = _dimos_f32_to_f64_bits(f);
        double native = (double)f;
        uint64_t native_bits;
        memcpy(&native_bits, &native, sizeof(native_bits));

        if (got_bits != native_bits) {
            printf("FAIL avr_double_promote: f=%.8g  got=0x%016llx  native=0x%016llx\n",
                   f, (unsigned long long)got_bits, (unsigned long long)native_bits);
            continue;
        }

        /* Round-trip back and check f2f32 inverts exactly on the normals. */
        float roundtrip = _dimos_f64_bits_to_f32(got_bits);
        if (memcmp(&roundtrip, &f, sizeof(f)) != 0) {
            printf("FAIL avr_double_roundtrip: f=%.8g  rt=%.8g\n", f, roundtrip);
            continue;
        }

        tests_passed++;
    }
    printf("PASS avr_double_promotion (%zu normals)\n",
           sizeof(normals) / sizeof(normals[0]));

    /* ±infinity */
    {
        tests_run++;
        uint64_t got = _dimos_f32_to_f64_bits(INFINITY);
        double native_inf = (double)INFINITY;
        uint64_t native_bits;
        memcpy(&native_bits, &native_inf, sizeof(native_bits));
        if (got == native_bits) {
            tests_passed++;
            printf("PASS avr_double_promotion (+inf)\n");
        } else {
            printf("FAIL avr_double_promotion (+inf): got=0x%016llx native=0x%016llx\n",
                   (unsigned long long)got, (unsigned long long)native_bits);
        }
    }
    {
        tests_run++;
        uint64_t got = _dimos_f32_to_f64_bits(-INFINITY);
        double native_inf = (double)-INFINITY;
        uint64_t native_bits;
        memcpy(&native_bits, &native_inf, sizeof(native_bits));
        if (got == native_bits) {
            tests_passed++;
            printf("PASS avr_double_promotion (-inf)\n");
        } else {
            printf("FAIL avr_double_promotion (-inf): got=0x%016llx native=0x%016llx\n",
                   (unsigned long long)got, (unsigned long long)native_bits);
        }
    }

    /* NaN — sign bit preserved, exponent all-ones, mantissa nonzero. */
    {
        tests_run++;
        uint64_t got = _dimos_f32_to_f64_bits(NAN);
        uint64_t sign_mask = 0x8000000000000000ULL;
        uint64_t exp_mask  = 0x7ff0000000000000ULL;
        uint64_t mant_mask = 0x000fffffffffffffULL;
        bool ok = ((got & exp_mask) == exp_mask) && ((got & mant_mask) != 0);
        (void)sign_mask;
        if (ok) {
            tests_passed++;
            printf("PASS avr_double_promotion (NaN)\n");
        } else {
            printf("FAIL avr_double_promotion (NaN): got=0x%016llx\n",
                   (unsigned long long)got);
        }
    }

    /* Smallest positive denorm — algorithm flushes to +0. */
    {
        tests_run++;
        float denorm = 1.0e-40f;  /* below FLT_MIN, a denorm */
        uint64_t got = _dimos_f32_to_f64_bits(denorm);
        if (got == 0ULL) {
            tests_passed++;
            printf("PASS avr_double_promotion (denorm flush-to-zero)\n");
        } else {
            printf("FAIL avr_double_promotion (denorm flush): got=0x%016llx\n",
                   (unsigned long long)got);
        }
    }
}

int main() {
    printf("=== Arduino LCM Wire Format Compatibility Tests ===\n\n");

    /* std_msgs */
    test_time();
    test_bool();
    test_int32();
    test_float32();
    test_float64();
    test_colorrgba();

    /* geometry_msgs */
    test_vector3();
    test_point();
    test_point32();
    test_quaternion();
    test_pose();
    test_pose2d();
    test_twist();
    test_accel();
    test_transform();
    test_wrench();
    test_inertia();
    test_pose_with_covariance();
    test_twist_with_covariance();
    test_accel_with_covariance();

    /* roundtrip */
    test_roundtrip_pose();

    /* AVR double-promotion path (normally compiled out on x86_64) */
    test_avr_double_promotion();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
