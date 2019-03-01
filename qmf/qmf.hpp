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

#ifndef qmf_qmf_hpp_included_
#define qmf_qmf_hpp_included_

#include <iostream>

#include <boost/filesystem.hpp>

#include "geometry/mesh.hpp"

#include "geo/srsdef.hpp"

#include "miniball/miniball.hpp"

namespace qmf {

typedef miniball::MinimumBoundingSphere3_<double> MinimumBoundingSphere;

/** Un-quantized mesh.
 */
struct Mesh {
    /** Mesh itself, in geographic SRS.
     */
    geometry::Mesh mesh;

    /** Mesh extents in geographic SRS.
     */
    math::Extents2 extents;

    /** Center in geocentric coordinates.
     */
    math::Point3 center;

    /** Minimum bounding sphere
     */
    MinimumBoundingSphere mbs;

    /** Horizon occlusion point, in geocentric coordinates.
     */
    math::Point3 hop;
};

/** Calculates mesh derived data. Srs is SRS of vertices in mesh.
 */
void calculateDerivedData(Mesh &mesh, const geo::SrsDefinition &srs);

Mesh load(const math::Extents2 &extents, std::istream &is
          , const boost::filesystem::path &path = "unknown");

Mesh load(const math::Extents2 &extents, const boost::filesystem::path &path);

void save(const Mesh &mesh, std::ostream &os
          , const boost::filesystem::path &path = "unknown");

void save(const Mesh &mesh, const boost::filesystem::path &path);

} // namespace qmf

#endif // qmf_qmf_hpp_included_
