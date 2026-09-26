#ifndef slic3r_STLRepair_hpp_
#define slic3r_STLRepair_hpp_

#include "libslic3r.h"
#include "TriangleMesh.hpp"
#include <string>

namespace Slic3r {

// Result of a repair operation.
struct STLRepairResult {
    int degenerate_facets_removed = 0;
    int invalid_faces_removed     = 0;
    int vertices_merged           = 0;
    int normals_fixed             = 0;
    int facets_reversed           = 0;
    int holes_filled              = 0;
    bool orientation_fixed        = false;

    bool anything_repaired() const {
        return degenerate_facets_removed > 0 || invalid_faces_removed > 0 ||
               vertices_merged > 0 || normals_fixed > 0 || facets_reversed > 0 ||
               holes_filled > 0 || orientation_fixed;
    }

    std::string summary() const;
};

// Platform-independent mesh repair for indexed_triangle_set.
// Uses only C++ standard library, Eigen, and the bundled admesh/CGAL libraries.
class STLRepair {
public:
    struct Options {
        // Remove faces with duplicate vertex indices or zero area.
        bool remove_degenerate_faces = true;
        // Remove faces referencing out-of-bounds vertex indices.
        bool remove_invalid_faces = true;
        // Merge vertices that are within tolerance of each other.
        bool merge_vertices = true;
        // Vertex merge tolerance (distance squared).
        float merge_tolerance_sq = 1e-12f;
        // Recalculate face normals from geometry.
        bool fix_normals = true;
        // Ensure consistent face orientation (positive volume).
        bool fix_orientation = true;
        // Fill holes using CGAL (requires CGAL).
        bool fill_holes = false;
        // Maximum merge tolerance increment steps (for multi-pass).
        int merge_iterations = 2;
    };

    STLRepair() = default;
    explicit STLRepair(Options opts) : m_opts(std::move(opts)) {}

    // Repair an indexed_triangle_set in place.
    // Returns a summary of what was repaired.
    STLRepairResult repair(indexed_triangle_set &its) const;

    // Repair a TriangleMesh in place.
    STLRepairResult repair(TriangleMesh &mesh) const;

    // Convenience: repair and return a new mesh, leaving the original untouched.
    TriangleMesh repair_copy(const TriangleMesh &mesh) const;

    const Options& options() const { return m_opts; }
    void set_options(Options opts) { m_opts = std::move(opts); }

private:
    Options m_opts;

    // Internal repair steps operating on indexed_triangle_set.
    int  remove_degenerate_faces(indexed_triangle_set &its) const;
    int  remove_invalid_faces(indexed_triangle_set &its) const;
    int  merge_duplicate_vertices(indexed_triangle_set &its, float tolerance_sq) const;
    void fix_normals(indexed_triangle_set &its) const;
    bool fix_orientation(indexed_triangle_set &its) const;
    int  fill_holes_cgal(indexed_triangle_set &its) const;
};

} // namespace Slic3r

#endif // slic3r_STLRepair_hpp_
