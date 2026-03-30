#include "uat_protocol.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <vector>

extern "C" {
#include "fec/rs.h"
}

bool test_rs_decode(unsigned seed, int trials, int symsize, int gfpoly, int fcr, int prim, int nroots, int pad)
{
    srand(seed);

    void *rs = ::init_rs_char(symsize, gfpoly, fcr, prim, nroots, pad);

    int NN = (1 << symsize) - 1;
    int blocklen = NN - pad;
    int datalen = blocklen - nroots;
    bool success = true;

    std::vector<unsigned char> test_block(blocklen, 0);
    std::vector<unsigned char> working_block(blocklen, 0);

    for (int trial = 0; trial < trials; ++trial) {
        /* build random test data, encode it */
        for (int i = 0; i < datalen; ++i)
            test_block[i] = std::rand() & NN;

        ::encode_rs_char(rs, test_block.data(), test_block.data() + datalen);

        /* test errors */
        for (int n_errors = 0; n_errors <= nroots; ++n_errors) {
            working_block = test_block;

            std::vector<int> error_val(blocklen, 0);
            std::vector<int> error_pos;
            std::vector<int> corrected_pos(nroots, 0);

            /* generate errors */
            for (int i = 0; i < n_errors; ++i) {
                int loc;
                do {
                    loc = std::rand() % blocklen;
                } while (error_val[loc]);

                int bits;
                do {
                    bits = std::rand() & NN;
                } while (!bits);

                error_val[loc] = bits;
                working_block[loc] ^= bits;
                error_pos.push_back(loc);
            }

            /* try to decode */
            int n_corrected = ::decode_rs_char(rs, working_block.data(), corrected_pos.data(), 0);

            if (n_errors > nroots / 2) {
                /* exceeded error correction capacity */
                if (n_corrected >= 0) {
                    std::cerr << "RS(" << blocklen << "," << datalen << ") (seed: " << seed << " trial: " << trial << " errors: " << n_errors << ")"
                              << " returned success, but should have failed" << std::endl;
                    success = false;
                }
            } else {
                if (n_corrected != n_errors) {
                    std::cerr << "RS(" << blocklen << "," << datalen << ") (seed: " << seed << " trial: " << trial << " errors: " << n_errors << ")"
                              << " claimed to correct " << n_corrected << " errors" << std::endl;
                    success = false;
                }
                if (working_block != test_block) {
                    std::cerr << "RS(" << blocklen << "," << datalen << ") (seed: " << seed << " trial: " << trial << " errors: " << n_errors << ")"
                              << " data wasn't corrected correctly" << std::endl;
                    success = false;
                }

                for (int i = 0; i < n_corrected; ++i) {
                    auto found = std::find(error_pos.begin(), error_pos.end(), corrected_pos[i]);
                    if (found == error_pos.end()) {
                        std::cerr << "RS(" << blocklen << "," << datalen << ") (seed: " << seed << " trial: " << trial << " errors: " << n_errors << ")"
                                  << " corrected symbol at position " << corrected_pos[i] << " which was not in error" << std::endl;
                        success = false;
                    } else {
                        error_pos.erase(found);
                    }
                }

                if (!error_pos.empty()) {
                    std::cerr << "RS(" << blocklen << "," << datalen << ") (seed: " << seed << " trial: " << trial << " errors: " << n_errors << ")"
                              << " missed errors at positions [";
                    for (int pos : error_pos)
                        std::cerr << pos;
                    std::cerr << "]" << std::endl;
                    success = false;
                }
            }
        }
    }

    ::free_rs_char(rs);
    return success;
}

int main(int argc, char **argv)
{
    bool success = true;

    success = test_rs_decode(/* seed */ 1,
                             /* trials */ 10000,
                             /* symsize */ 8,
                             /* gfpoly */ flightaware::uat::fec::DOWNLINK_SHORT_POLY,
                             /* fcr */ 120,
                             /* prim */ 1,
                             /* nroots */ flightaware::uat::fec::DOWNLINK_SHORT_ROOTS,
                             /* pad */ flightaware::uat::fec::DOWNLINK_SHORT_PAD) && success;

    success = test_rs_decode(/* seed */ 1,
                             /* trials */ 10000,
                             /* symsize */ 8,
                             /* gfpoly */ flightaware::uat::fec::DOWNLINK_LONG_POLY,
                             /* fcr */ 120,
                             /* prim */ 1,
                             /* nroots */ flightaware::uat::fec::DOWNLINK_LONG_ROOTS,
                             /* pad */ flightaware::uat::fec::DOWNLINK_LONG_PAD) && success;

    success = test_rs_decode(/* seed */ 1,
                             /* trials */ 10000,
                             /* symsize */ 8,
                             /* gfpoly */ flightaware::uat::fec::UPLINK_BLOCK_POLY,
                             /* fcr */ 120,
                             /* prim */ 1,
                             /* nroots */ flightaware::uat::fec::UPLINK_BLOCK_ROOTS,
                             /* pad */ flightaware::uat::fec::UPLINK_BLOCK_PAD) && success;

    return success ? EXIT_SUCCESS : EXIT_FAILURE;
}
