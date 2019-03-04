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
#include "utility/enum-io.hpp"

#include "geometry/meshop.hpp"

#include "service/cmdline.hpp"

#include "geo/csconvertor.hpp"

#include "qmf/qmf.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace {

UTILITY_GENERATE_ENUM(Format,
                      ((qmf))
                      ((obj))
                      ((ply))
                      )

class Convert : public service::Cmdline
{
public:
    Convert()
        : service::Cmdline("qmf-convert", BUILD_TARGET_VERSION)
        , inputFormat_(Format::qmf)
        , outputFormat_(Format::ply)
        , geocent_(true)
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

    void saveMesh(qmf::Mesh mesh) const;

    void saveMesh(const geometry::Mesh &mesh) const;

    fs::path input_;
    Format inputFormat_;
    fs::path output_;
    Format outputFormat_;
    math::Extents2 extents_;
    bool geocent_;
};

void Convert::configuration(po::options_description &cmdline
                        , po::options_description &config
                        , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("input", po::value(&input_)->required()
         , "Input file.")
        ("output", po::value(&output_)->required()
         , "Output file.")
        ("extents", po::value(&extents_)->required()
         , "Tile extents in geographic SRS.")
        ("inputFormat"
         , po::value(&inputFormat_)->default_value(inputFormat_)
         , "Input format; qmf/ply/obj")
        ("outputFormat"
         , po::value(&outputFormat_)->default_value(outputFormat_)
         , "Output format; qmf/ply/obj")
        ("geocent"
         , po::value(&geocent_)->default_value(geocent_)
         , "Non-qmf meshes in geocentric coordinates if true.")
        ;

    pd.add("input", 1)
        .add("output", 1)
        .add("extents", 1)
        ;

    (void) config;
}

void Convert::configure(const po::variables_map &vars)
{
    (void) vars;
}

bool Convert::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(qmf-convert
usage
    qmf-convert INPUT OUTPUT [OPTIONS]

)RAW";
    }
    return false;
}

geometry::Mesh inGeocent(geometry::Mesh mesh)
{
    const auto src(geo::SrsDefinition::longlat());
    const auto dst(geo::geocentric(src));
    mesh.vertices = geo::CsConvertor(src, dst)(mesh.vertices);
    return mesh;
}

geometry::Mesh fromGeocent(geometry::Mesh mesh)
{
    const auto dst(geo::SrsDefinition::longlat());
    const auto src(geo::geocentric(dst));
    mesh.vertices = geo::CsConvertor(src, dst)(mesh.vertices);
    return mesh;
}

void Convert::saveMesh(qmf::Mesh mesh) const
{
    switch (outputFormat_) {
    case Format::qmf: {
        // recalculate metadata
        qmf::calculateDerivedData(mesh, geo::SrsDefinition::longlat());
        qmf::save(mesh, output_);
        break;
    }

    case Format::obj:
        geometry::saveAsObj(geocent_ ? inGeocent(mesh.mesh) : mesh.mesh
                            , output_, "");
        break;

    case Format::ply:
        geometry::saveAsPly(geocent_ ? inGeocent(mesh.mesh) : mesh.mesh
                            , output_);
        break;
    }
}

void Convert::saveMesh(const geometry::Mesh &mesh) const
{
    qmf::Mesh qmesh;
    qmesh.mesh = geocent_ ? fromGeocent(mesh) : mesh;
    qmesh.extents = extents_;
    saveMesh(qmesh);
}

int Convert::run()
{
    // expecting terrain file, will detect mesh type later
    switch (inputFormat_) {
    case Format::qmf:
        saveMesh(qmf::load(extents_, input_));
        break;

    case Format::obj:
        abort();
        break;

    case Format::ply:
        saveMesh(geometry::loadPly(input_));
        break;
    }

    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return Convert()(argc, argv);
}
