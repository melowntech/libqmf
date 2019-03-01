/**
 * Copyright (c) 2019 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdlib>
#include <iostream>

#include "utility/buildsys.hpp"
#include "utility/gccversion.hpp"

#include "service/cmdline.hpp"

#include "qmf/qmf.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace {

class Encode : public service::Cmdline
{
public:
    Encode()
        : service::Cmdline("qmf-encode", BUILD_TARGET_VERSION)
    {}

private:
    virtual void configuration(po::options_description &cmdline
                               , po::options_description &config
                               , po::positional_options_description &pd)
        UTILITY_OVERRIDE;

    virtual void configure(const po::variables_map &vars)
        UTILITY_OVERRIDE;

    virtual bool help(std::ostream &out, const std::string &what) const
        UTILITY_OVERRIDE;

    virtual int run() UTILITY_OVERRIDE;

    fs::path input_;
    fs::path output_;
    math::Extents2 extents_;
};

void Encode::configuration(po::options_description &cmdline
                        , po::options_description &config
                        , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("input", po::value(&input_)->required()
         , "Input OBJ file.")
        ("output", po::value(&output_)->required()
         , "Output terrain file.")
        ("extents", po::value(&output_)->required()
         , "Tile extents in geographic SRS.")
        ;

    pd.add("input", 1)
        .add("output", 1)
        .add("extents", 1);

    (void) config;
}

void Encode::configure(const po::variables_map &vars)
{
    (void) vars;
}

bool Encode::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(qmf-encode
usage
    qmf-encode INPUT OUTPUT [OPTIONS]

)RAW";
    }
    return false;
}

int Encode::run()
{
    // expecting terrain file, will detect mesh type later
    auto mesh(qmf::load(extents_, input_));

    qmf::save(mesh, output_);

    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return Encode()(argc, argv);
}
