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


#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"

#include "math/geometry.hpp"
#include "geo/csconvertor.hpp"

#include "miniball/miniball.hpp"

#include "qmf.hpp"

namespace qmf {

namespace {

namespace bin = utility::binaryio;

/** Tile header
 */
struct Header {
    /** Tile center in geocentric coordinates.
     */
    math::Point3d center;

    /** Tile height range (min, max).
     */
    math::Point2f heightRange;

    /** Tile minimum bounding sphere (x, y, z, r)
     */
    math::Point4d mbs;

    /** Horizon occlusion point.
     *  See http://cesiumjs.org/2013/04/25/Horizon-culling/ .
     */
    // math::Point3d hop;
    math::Point3d hop;
};

void load(std::istream &is, Header &h)
{
    bin::read(is, h.center(0));
    bin::read(is, h.center(1));
    bin::read(is, h.center(2));

    bin::read(is, h.heightRange(0));
    bin::read(is, h.heightRange(1));

    bin::read(is, h.mbs(0));
    bin::read(is, h.mbs(1));
    bin::read(is, h.mbs(2));
    bin::read(is, h.mbs(3));

    bin::read(is, h.hop(0));
    bin::read(is, h.hop(1));
    bin::read(is, h.hop(2));
}

inline std::int16_t unzigzag(std::uint16_t value)
{
    return ((value >> 1) ^ (-(value & 1)));
}

void decodeVertices(const math::Extents2 &extents
                    , const math::Point2d &heightRange
                    , std::istream &is
                    , math::Points3d &vertices)
{
    const auto vc(bin::read<std::uint32_t>(is));
    LOG(info4) << "vertex count: " << vc;

    typedef std::vector<uint16_t> VertexBuffer;
    const auto loadVcBuffer([&]() -> VertexBuffer
    {
        VertexBuffer buf;
        buf.reserve(vc);
        for (auto left(vc); left; --left) {
            buf.push_back(bin::read<uint16_t>(is));
        }
        return buf;
    });

    vertices.clear();
    vertices.reserve(vc);

    const auto uBuf(loadVcBuffer());
    const auto vBuf(loadVcBuffer());
    const auto hBuf(loadVcBuffer());

    const auto es(math::size(extents));
    const auto height(heightRange(1) - heightRange(0));

    const double quntizedRange
        (std::numeric_limits<std::uint16_t>::max());
    const auto remap([&](double offset, double range
                         , std::uint16_t value) -> double
    {
        return offset + (range * value) / quntizedRange;
    });

    std::uint16_t u(0), v(0), h(0);
    for (std::uint32_t i(0); i != vc; ++i) {
        u += unzigzag(uBuf[i]);
        v += unzigzag(vBuf[i]);
        h += unzigzag(hBuf[i]);

        vertices.emplace_back(remap(extents.ll(0), es.width, u)
                              , remap(extents.ll(1), es.height, v)
                              , remap(heightRange(0), height, h));
    }
}

template <typename T>
void decodeIndices(std::istream &is, geometry::Face::list &faces)
{
    auto tc(bin::read<std::uint32_t>(is));
    LOG(info4) << "triangle count: " << tc;
    faces.clear();
    faces.reserve(tc);

    T highest(0);
    const auto decodeIndex([&]()
    {
        auto code(bin::read<T>(is));
        geometry::Face::index_type index(highest - code);
        if (!code) { ++highest; }
        return index;
    });

    for (auto i(tc); i; --i) {
        const auto a(decodeIndex());
        const auto b(decodeIndex());
        const auto c(decodeIndex());
        faces.emplace_back(a, b, c);
    }
}

void decodeIndices(std::istream &is, geometry::Mesh &mesh)
{
    if (mesh.vertices.size() > std::numeric_limits<std::uint16_t>::max()) {
        const auto offset(3 * sizeof(std::uint16_t) * mesh.vertices.size());
        if (offset % 4) {
            // enforce 4-byte alignment
            bin::read<std::uint16_t>(is);
        }

        decodeIndices<std::uint32_t>(is, mesh.faces);
    } else {
        decodeIndices<std::uint16_t>(is, mesh.faces);
    }
}

} // namespace

void calculateDerivedData(Mesh &mesh, const geo::SrsDefinition &srs)
{
    geo::CsConvertor conv(srs, geo::geocentric(srs));
    auto world(conv(mesh.mesh.vertices));

    mesh.center = math::center(math::computeExtents(world));

    // TODO: calculate MBS
    // TODO: calculate HOP
}

Mesh load(const math::Extents2 &extents, std::istream &is
          , const boost::filesystem::path &path)
{
    (void) path;
    Header header;
    load(is, header);

    LOG(info4) << std::fixed << "center: " << header.center;
    LOG(info4) << std::fixed << "heightRange: " << header.heightRange;
    LOG(info4) << std::fixed << "mbs: " << header.mbs;
    LOG(info4) << std::fixed << "hop: " << header.hop;

    Mesh mesh;
    mesh.extents = extents;
    mesh.center = header.center;
    mesh.hop = header.hop;

    auto &m(mesh.mesh);
    decodeVertices(extents, header.heightRange, is, m.vertices);
    decodeIndices(is, m);

    return mesh;
}

Mesh load(const math::Extents2 &extents, const boost::filesystem::path &path)
{
    LOG(info1) << "Loading quantized mesh from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(path.string(), std::ios_base::in);
    const auto mesh(load(extents, f, path));
    f.close();
    return mesh;
}

void save(const Mesh &mesh, std::ostream &os
          , const boost::filesystem::path &path)
{
    (void) mesh;
    (void) os;
    (void) path;
}

void save(const Mesh &mesh, const boost::filesystem::path &path)
{
    LOG(info1) << "Saving quantized mesh " << path  << ".";
    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(path.string(), std::ios_base::out);
    save(mesh, f, path);
    f.close();
}


} // namespace qmf