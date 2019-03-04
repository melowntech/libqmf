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

#include "math/math.hpp"
#include "math/geometry.hpp"
#include "geo/csconvertor.hpp"

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

    /** Tile minimum bounding sphere.
     */
    miniball::MinimumBoundingSphere3_<double> mbs;

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

    bin::read(is, h.mbs.center(0));
    bin::read(is, h.mbs.center(1));
    bin::read(is, h.mbs.center(2));
    bin::read(is, h.mbs.radius);

    bin::read(is, h.hop(0));
    bin::read(is, h.hop(1));
    bin::read(is, h.hop(2));
}

void save(std::ostream &os, const Header &h)
{
    bin::write(os, h.center(0));
    bin::write(os, h.center(1));
    bin::write(os, h.center(2));

    bin::write(os, h.heightRange(0));
    bin::write(os, h.heightRange(1));

    bin::write(os, h.mbs.center(0));
    bin::write(os, h.mbs.center(1));
    bin::write(os, h.mbs.center(2));
    bin::write(os, h.mbs.radius);

    bin::write(os, h.hop(0));
    bin::write(os, h.hop(1));
    bin::write(os, h.hop(2));
}

const int QuantizedMax(32767);
typedef std::vector<uint16_t> VertexBuffer;

inline std::int16_t unzigzag(std::uint16_t value)
{
    return (std::int16_t(value >> 1) ^ (-std::int16_t(value & 1)));
}

inline std::uint16_t zigzag(std::int16_t value)
{
    return ((value >> 15) ^ (value << 1));
}

void decodeVertices(const math::Extents2 &extents
                    , const math::Point2d &heightRange
                    , std::istream &is
                    , math::Points3d &vertices)
{
    const auto vc(bin::read<std::uint32_t>(is));

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

    const auto unquantize([&](double offset, double range
                              , std::uint16_t value) -> double
    {
        return (offset + (range * value) / double(QuantizedMax));
    });

    std::int16_t u(0), v(0), h(0);
    for (std::uint32_t i(0); i != vc; ++i) {
        u += unzigzag(uBuf[i]);
        v += unzigzag(vBuf[i]);
        h += unzigzag(hBuf[i]);

        const auto x(unquantize(extents.ll(0), es.width, u));
        const auto y(unquantize(extents.ll(1), es.height, v));
        const auto z(unquantize(heightRange(0), height, h));
        vertices.emplace_back(x, y, z);
    }
}

template <typename T>
void decodeIndices(std::istream &is, geometry::Face::list &faces)
{
    auto tc(bin::read<std::uint32_t>(is));
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
    mesh.mbs = miniball::minimumBoundingSphere(world);

    // calculate HOP
    const auto ellipsoid(geo::ellipsoid(srs));

    const auto &scale([&](const math::Point3 &v)
    {
        return math::Point3
            (v(0) / ellipsoid(0), v(1) / ellipsoid(1), v(2) / ellipsoid(2));
    });

    const auto scaledCenter(scale(mesh.center));

    double pScale(0);
    for (const auto &wp : world) {
        const auto p(scale(wp));
        auto magnitudeSquared
            (math::sqr(p(0)) + math::sqr(p(1)) + math::sqr(p(2)));
        auto magnitude(std::sqrt(magnitudeSquared));
        const math::Point3 direction(p / magnitude);

        magnitudeSquared = std::max(magnitudeSquared, 1.0);
        magnitude = std::max(magnitude, 1.0);

        const auto cosAlpha(inner_prod(direction, scaledCenter));
        const auto sinAlpha(math::length(math::crossProduct(p, scaledCenter)));
        const auto cosBeta(1.0 / magnitude);
        const auto sinBeta(std::sqrt(1.0 - magnitudeSquared) * cosBeta);

        pScale = std::max
            (pScale, 1.0 / (cosAlpha * cosBeta - sinAlpha * sinBeta));
    }

    mesh.hop = scaledCenter * pScale;
}

Mesh load(const math::Extents2 &extents, std::istream &is
          , const boost::filesystem::path &path)
{
    (void) path;
    Header header;
    load(is, header);

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
    f.open(path.string(), (std::ios_base::in | std::ios_base::binary));
    const auto mesh(load(extents, f, path));
    f.close();
    return mesh;
}

typedef geometry::Face::index_type Index;
typedef std::vector<Index> IndexList;

template <typename T>
void encodeIndices(std::ostream &os, const geometry::Face::list &faces
                   , const IndexList &vertexOrder)
{
    // write face count
    bin::write(os, std::uint32_t(faces.size()));

    T highest(0);
    const auto &writeIndex([&](T index)
    {
        const T delta(highest - index);
        if (!delta) { ++highest; }
        bin::write(os, delta);
    });

    for (const auto &face : faces) {
        writeIndex(vertexOrder[face.a]);
        writeIndex(vertexOrder[face.b]);
        writeIndex(vertexOrder[face.c]);
    }
}

template<typename T>
void writeSkirt(std::ostream &os, const IndexList &indices)
{
    bin::write(os, std::uint32_t(indices.size()));
    for (const auto &index : indices) {
        bin::write(os, T(index));
    }
}

void encodeIndices(std::ostream &os, const geometry::Mesh &mesh
                   , const IndexList &vertexOrder
                   , const IndexList &west, const IndexList &south
                   , const IndexList &east, const IndexList &north)
{
    if (vertexOrder.size() > std::numeric_limits<std::uint16_t>::max()) {
        const auto offset(3 * sizeof(std::uint16_t) * vertexOrder.size());
        if (offset % 4) {
            // enforce 4-byte alignment
            bin::write(os, std::uint16_t(0));
        }
        encodeIndices<std::uint32_t>(os, mesh.faces, vertexOrder);
        writeSkirt<std::uint32_t>(os, west);
        writeSkirt<std::uint32_t>(os, south);
        writeSkirt<std::uint32_t>(os, east);
        writeSkirt<std::uint32_t>(os, north);
    } else {
        encodeIndices<std::uint16_t>(os, mesh.faces, vertexOrder);
        writeSkirt<std::uint16_t>(os, west);
        writeSkirt<std::uint16_t>(os, south);
        writeSkirt<std::uint16_t>(os, east);
        writeSkirt<std::uint16_t>(os, north);
    }
}

void save(const Mesh &mesh, std::ostream &os
          , const boost::filesystem::path &path)
{
    const auto &m(mesh.mesh);

    // serialize header
    Header header;
    {
        // compute height range
        const auto ge(math::computeExtents(m.vertices));
        header.heightRange(0) = ge.ll(2);
        header.heightRange(1) = ge.ur(2);

        header.center = mesh.center;
        header.mbs = mesh.mbs;
        header.hop = mesh.hop;
    }
    save(os, header);

    /** Quantizes value in range [0, QuantizedMax]. Larger values are clipped.
     */
    const auto &quantize([&](double offset, double range
                             , double value) -> int
    {
        // range can be zero!
        const auto tmp((value - offset) * double(QuantizedMax));
        const int res(range ? (tmp / range) : tmp);
        if (res < 0) { return 0; }
        if (res > QuantizedMax) { return QuantizedMax; }
        return res;
    });

    const auto saveVcBuffer([&](const VertexBuffer buf)
    {
        for (const auto &value : buf) {
            bin::write(os, value);
        }
    });

    const auto invalid(Index(-1));
    IndexList vertexOrder(m.vertices.size(), invalid);
    IndexList west, south, east, north;

    const auto es(math::size(mesh.extents));
    const auto height(header.heightRange(1) - header.heightRange(0));

    Index outIndex(0);
    VertexBuffer uBuf, vBuf, hBuf;
    int pu(0), pv(0), ph(0);
    const auto &serializeVertex([&](Index index)
    {
        auto &mapped(vertexOrder[index]);
        if (mapped != invalid) { return; }
        mapped = outIndex++;

        const auto &vertex(m.vertices[index]);

        const auto u(quantize(mesh.extents.ll(0), es.width, vertex(0)));
        const auto v(quantize(mesh.extents.ll(1), es.height, vertex(1)));
        const auto h(quantize(header.heightRange(0), height, vertex(2)));

        if (!u) { west.push_back(mapped); }
        if (u == QuantizedMax) { east.push_back(mapped); }
        if (!v) { south.push_back(mapped); }
        if (v == QuantizedMax) { north.push_back(mapped); }

        uBuf.push_back(zigzag(u - pu));
        vBuf.push_back(zigzag(v - pv));
        hBuf.push_back(zigzag(h - ph));
        pu = u;
        pv = v;
        ph = h;
    });

    // encode vertices
    for (const auto &face : m.faces) {
        serializeVertex(face.a);
        serializeVertex(face.b);
        serializeVertex(face.c);
    }

    // write vertex count
    bin::write(os, std::uint32_t(m.vertices.size()));
    // write individual vertex components
    saveVcBuffer(uBuf);
    saveVcBuffer(vBuf);
    saveVcBuffer(hBuf);

    // write faces and skirt indices
    encodeIndices(os, m, vertexOrder, west, south, east, north);
    (void) path;
}

void save(const Mesh &mesh, const boost::filesystem::path &path)
{
    LOG(info1) << "Saving quantized mesh " << path  << ".";
    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(path.string(), (std::ios_base::out | std::ios_base::trunc
                           | std::ios_base::binary));
    save(mesh, f, path);
    f.close();
}

} // namespace qmf
