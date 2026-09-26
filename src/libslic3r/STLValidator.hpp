#ifndef slic3r_STLValidator_hpp_
#define slic3r_STLValidator_hpp_

#include "libslic3r.h"
#include <admesh/stl.h>
#include <string>
#include <vector>

namespace Slic3r {

class TriangleMesh;

// Describes a single validation issue found in a mesh.
struct STLValidationIssue {
    enum class Severity { Warning, Error, Critical };
    enum class Type {
        EmptyMesh,
        DegenerateFacet,
        NonManifoldEdge,
        OpenEdge,
        InconsistentNormal,
        FlippedFace,
        NaNVertex,
        InfVertex,
        ZeroVolumeBoundingBox,
        DisconnectedComponent,
        InvalidFaceIndex,
        ZeroAreaFacet,
    };

    Type     type;
    Severity severity;
    size_t   facet_index;  // Index of the offending facet, or SIZE_MAX if global
    std::string message;

    STLValidationIssue(Type t, Severity s, size_t fi, std::string msg)
        : type(t), severity(s), facet_index(fi), message(std::move(msg)) {}
};

// Result of a validation pass.
struct STLValidationResult {
    std::vector<STLValidationIssue> issues;

    bool valid() const { return issues.empty(); }
    bool has_errors() const;
    bool has_critical() const;
    size_t error_count() const;
    size_t warning_count() const;

    // Summary string for logging/UI.
    std::string summary() const;
};

// Comprehensive validation of an indexed_triangle_set.
// Platform-independent, uses only C++ standard library and Eigen.
class STLValidator {
public:
    struct Options {
        bool check_degenerate_facets   = true;
        bool check_manifold_edges      = true;
        bool check_open_edges          = true;
        bool check_normals             = true;
        bool check_nan_vertices        = true;
        bool check_face_indices        = true;
        bool check_disconnected        = true;
        bool check_bounding_box        = true;
        // Threshold for considering two vertices as the same position.
        float merge_tolerance          = 1e-6f;
        // Maximum number of issues per type before stopping (0 = unlimited).
        size_t max_issues_per_type     = 1000;
    };

    STLValidator() = default;
    explicit STLValidator(Options opts) : m_opts(std::move(opts)) {}

    // Validate an indexed_triangle_set.
    STLValidationResult validate(const indexed_triangle_set &its) const;

    // Validate a TriangleMesh (includes stats-based checks).
    STLValidationResult validate(const TriangleMesh &mesh) const;

    const Options& options() const { return m_opts; }
    void set_options(Options opts) { m_opts = std::move(opts); }

private:
    Options m_opts;

    void check_empty(const indexed_triangle_set &its, STLValidationResult &result) const;
    void check_face_indices(const indexed_triangle_set &its, STLValidationResult &result) const;
    void check_degenerate_facets(const indexed_triangle_set &its, STLValidationResult &result) const;
    void check_nan_vertices(const indexed_triangle_set &its, STLValidationResult &result) const;
    void check_manifold_edges(const indexed_triangle_set &its, STLValidationResult &result) const;
    void check_open_edges(const indexed_triangle_set &its, STLValidationResult &result) const;
    void check_normals(const indexed_triangle_set &its, STLValidationResult &result) const;
    void check_bounding_box(const indexed_triangle_set &its, STLValidationResult &result) const;
    void check_disconnected(const indexed_triangle_set &its, STLValidationResult &result) const;
};

} // namespace Slic3r

#endif // slic3r_STLValidator_hpp_
