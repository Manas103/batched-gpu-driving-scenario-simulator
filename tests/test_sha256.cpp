#include <cstring>
#include <string>

#include "check.hpp"
#include "sim/sha256.hpp"

static std::string digest_of(const char* s) {
    sim::Sha256 h;
    h.update(s, strlen(s));
    return h.hexdigest();
}

int main() {
    // FIPS 180-4 / NIST published test vectors (cross-checked against
    // Python's hashlib.sha256 on this machine, not retyped from memory).
    CHECK(digest_of("") == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    CHECK(digest_of("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");

    // Multi-block input (longer than the 64-byte block size) against the
    // standard NIST two-block message vector.
    CHECK(digest_of("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq") ==
          "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");

    // Updating in two pieces must give the same digest as one piece.
    {
        sim::Sha256 h1;
        h1.update("abc", 3);
        std::string one_shot = h1.hexdigest();

        sim::Sha256 h2;
        h2.update("a", 1);
        h2.update("bc", 2);
        std::string two_shot = h2.hexdigest();
        CHECK(one_shot == two_shot);
    }

    CHECK_SUITE_END();
}
