#include "STLValidator.hpp"
#include "TriangleMesh.hpp"

#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include <boost/log/trivial.hpp>

namespace Slic3r {

// --- STLValidationResult ---

bool STLValidationResult::has_errors() const
{
    for (const auto &issue : issues)
        if (issue.severity == STLValidationIssue::Severity::Error ||
            issue.severity == STLValidationIssue::Severity::Critical)
            return true;
    return false;
}

bool STLValidationResult::has_critical() const
{
    for (const auto &issue : issues)
        if (issue.severity == STLValidationIssue::Severity::Critical)
            return true;
    return false;
}

size_t STLValidationResult::error_count() const
{
    size_t count = 0;
    for (const auto &issue : issues)
        if (issue.severity == STLValidationIssue::Severity::Error ||
            issue.severity == STLValidationIssue::Severity::Critical)
            ++count;
    return count;
}

size_t STLValidationResult::warning_count() const
{
    size_t count = 0;
    for (const auto &issue : issues)
        if (issue.severity == STLValidationIssue::Severity::Warning)
            ++count;
    return count;
}

std::string STLValidationResult::summary() const
{
    if (valid())
        return "Mesh is valid.";

    size_t errs   = error_count();
    size_t warns  = warning_count();
    size_t crits  = 0;
    for (const auto &issue : issues)
        if (issue.severity == STLValidationIssue::Severity::Critical)
            ++crits;

    std::string s;
    if (crits > 0) s += std::to_string(crits) + " critical";
    if (errs > 0) {
        if (!s.empty()) s += ", ";
        s += std::to_string(errs) + " error(s)";
    }
    if (warns > 0) {
        if (!s.empty()) s += ", ";
        s += std::to_string(warns) + " warning(s)";
    }
    return s;
}

// --- STLValidator ---

STLValidationResult STLValidator::validate(const indexed_triangle_set &its) const
{
    STLValidationResult result;

    check_empty(its, result);
    if (result.has_critical())
        return result;

    check_face_indices(its, result);
    check_nan_vertices(its, result);
    check_degenerate_facets(its, result);
    check_bounding_box(its, result);

    if (m_opts.check_manifold_edges || m_opts.check_open_edges)
        check_manifold_edges(its, result);

    check_normals(its, result);
    check_disconnected(its, result);

    return result;
}

STLValidationResult STLValidator::validate(const TriangleMesh &mesh) const
{
    return validate(mesh.its);
}

void STLValidator::check_empty(const indexed_triangle_set &its, STLValidationResult &result) const
{
    if (its.indices.empty() && its.vertices.empty())
        result.issues.emplace_back(
            STLValidationIssue::Type::EmptyMesh,
            STLValidationIssue::Severity::Critical,
            SIZE_MAX,
            "Mesh is empty: no faces or vertices.");
    else if (its.indices.empty())
        result.issues.emplace_back(
            STLValidationIssue::Type::EmptyMesh,
            STLValidationIssue::Severity::Critical,
            SIZE_MAX,
            "Mesh has vertices but no faces.");
    else if (its.vertices.empty())
        result.issues.emplace_back(
            STLValidationIssue::Type::EmptyMesh,
            STLValidationIssue::Severity::Critical,
            SIZE_MAX,
            "Mesh has faces but no vertices.");
}

void STLValidator::check_face_indices(const indexed_triangle_set &its, STLValidationResult &result) const
{
    const size_t num_verts = its.vertices.size();
    size_t count = 0;
    for (size_t fi = 0; fi < its.indices.size(); ++fi) {
        const auto &idx = its.indices[fi];
        for (int vi = 0; vi < 3; ++vi) {
            if (idx[vi] < 0 || size_t(idx[vi]) >= num_verts) {
                result.issues.emplace_back(
                    STLValidationIssue::Type::InvalidFaceIndex,
                    STLValidationIssue::Severity::Error,
                    fi,
                    "Face " + std::to_string(fi) + " has out-of-bounds vertex index " +
                    std::to_string(idx[vi]) + " (valid range: 0.." + std::to_string(num_verts - 1) + ").");
                if (++count >= m_opts.max_issues_per_type)
                    return;
            }
        }
    }
}

void STLValidator::check_nan_vertices(const indexed_triangle_set &its, STLValidationResult &result) const
{
    size_t count = 0;
    for (size_t fi = 0; fi < its.indices.size(); ++fi) {
        const auto &idx = its.indices[fi];
        for (int vi = 0; vi < 3; ++vi) {
            const auto &v = its.vertices[idx[vi]];
            if (std::isnan(v(0)) || std::isnan(v(1)) || std::isnan(v(2))) {
                result.issues.emplace_back(
                    STLValidationIssue::Type::NaNVertex,
                    STLValidationIssue::Severity::Error,
                    fi,
                    "Face " + std::to_string(fi) + " vertex " + std::to_string(vi) + " contains NaN.");
                if (++count >= m_opts.max_issues_per_type)
                    return;
            }
            if (std::isinf(v(0)) || std::isinf(v(1)) || std::isinf(v(2))) {
                result.issues.emplace_back(
                    STLValidationIssue::Type::InfVertex,
                    STLValidationIssue::Severity::Error,
                    fi,
                    "Face " + std::to_string(fi) + " vertex " + std::to_string(vi) + " contains Inf.");
                if (++count >= m_opts.max_issues_per_type)
                    return;
            }
        }
    }
}

void STLValidator::check_degenerate_facets(const indexed_triangle_set &its, STLValidationResult &result) const
{
    size_t count = 0;
    for (size_t fi = 0; fi < its.indices.size(); ++fi) {
        const auto &idx = its.indices[fi];
        // Check for collapsed triangles (2+ identical vertex indices).
        if (idx[0] == idx[1] || idx[1] == idx[2] || idx[0] == idx[2]) {
            result.issues.emplace_back(
                STLValidationIssue::Type::DegenerateFacet,
                STLValidationIssue::Severity::Warning,
                fi,
                "Face " + std::to_string(fi) + " is degenerate: has duplicate vertex indices.");
            if (++count >= m_opts.max_issues_per_type)
                return;
            continue;
        }
        // Check for zero-area triangles (collinear vertices).
        const auto &v0 = its.vertices[idx[0]];
        const auto &v1 = its.vertices[idx[1]];
        const auto &v2 = its.vertices[idx[2]];
        float area = 0.5f * (v1 - v0).cross(v2 - v0).norm();
        if (area < std::numeric_limits<float>::epsilon()) {
            result.issues.emplace_back(
                STLValidationIssue::Type::ZeroAreaFacet,
                STLValidationIssue::Severity::Warning,
                fi,
                "Face " + std::to_string(fi) + " has zero or near-zero area.");
            if (++count >= m_opts.max_issues_per_type)
                return;
        }
    }
}

void STLValidator::check_manifold_edges(const indexed_triangle_set &its, STLValidationResult &result) const
{
    // Build edge -> face count map.
    // Key: sorted (min, max) vertex index pair.
    struct EdgeHash {
        size_t operator()(const std::pair<int,int> &e) const {
            return std::hash<int>()(e.first) ^ (std::hash<int>()(e.second) << 16);
        }
    };
    std::unordered_map<std::pair<int,int>, int, EdgeHash> edge_count;
    edge_count.reserve(its.indices.size() * 3);

    for (size_t fi = 0; fi < its.indices.size(); ++fi) {
        const auto &idx = its.indices[fi];
        for (int ei = 0; ei < 3; ++ei) {
            int a = idx[ei];
            int b = idx[(ei + 1) % 3];
            auto key = a < b ? std::make_pair(a, b) : std::make_pair(b, a);
            ++edge_count[key];
        }
    }

    // Count open and non-manifold edges.
    size_t open_count = 0;
    size_t nonmanifold_count = 0;
    for (const auto &kv : edge_count) {
        if (kv.second == 1)
            ++open_count;
        else if (kv.second > 2)
            ++nonmanifold_count;
    }

    if (m_opts.check_open_edges && open_count > 0) {
        result.issues.emplace_back(
            STLValidationIssue::Type::OpenEdge,
            STLValidationIssue::Severity::Error,
            SIZE_MAX,
            "Mesh has " + std::to_string(open_count) + " open (boundary) edge(s). "
            "The mesh is not watertight.");
    }

    if (m_opts.check_manifold_edges && nonmanifold_count > 0) {
        result.issues.emplace_back(
            STLValidationIssue::Type::NonManifoldEdge,
            STLValidationIssue::Severity::Error,
            SIZE_MAX,
            "Mesh has " + std::to_string(nonmanifold_count) + " non-manifold edge(s) "
            "(shared by more than 2 faces).");
    }
}

void STLValidator::check_open_edges(const indexed_triangle_set &its, STLValidationResult &result) const
{
    // If manifold checking is already enabled, it handles open edges too.
    // Only run standalone open edge check if manifold checking is disabled.
    if (!m_opts.check_manifold_edges)
        check_manifold_edges(its, result);
}

void STLValidator::check_normals(const indexed_triangle_set &its, STLValidationResult &result) const
{
    // Face normals are recalculated from geometry during repair.
    // Here we only check for faces that would produce degenerate normals
    // (zero cross product), which is already covered by degenerate facet check.
    // Consistency checking (all normals pointing outward) requires a full
    // connected-component analysis and is done during repair.
}

void STLValidator::check_bounding_box(const indexed_triangle_set &its, STLValidationResult &result) const
{
    if (!m_opts.check_bounding_box)
        return;
    if (its.vertices.empty())
        return;

    Vec3f bmin = its.vertices.front();
    Vec3f bmax = its.vertices.front();
    for (const auto &v : its.vertices) {
        bmin = bmin.cwiseMin(v);
        bmax = bmax.cwiseMax(v);
    }

    Vec3f size = bmax - bmin;
    if (size(0) <= 0.0f || size(1) <= 0.0f || size(2) <= 0.0f) {
        result.issues.emplace_back(
            STLValidationIssue::Type::ZeroVolumeBoundingBox,
            STLValidationIssue::Severity::Error,
            SIZE_MAX,
            "Mesh bounding box has zero volume (" +
            std::to_string(size(0)) + " x " +
            std::to_string(size(1)) + " x " +
            std::to_string(size(2)) + "). The mesh is degenerate.");
    }
}

void STLValidator::check_disconnected(const indexed_triangle_set &its, STLValidationResult &result) const
{
    if (!m_opts.check_disconnected)
        return;
    if (its.indices.size() <= 1)
        return;

    // Union-Find to count connected components.
    const size_t n = its.indices.size();
    std::vector<size_t> parent(n);
    for (size_t i = 0; i < n; ++i)
        parent[i] = i;

    auto find = [&parent](size_t x) -> size_t {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    };
    auto unite = [&parent, &find](size_t a, size_t b) {
        a = find(a);
        b = find(b);
        if (a != b) parent[a] = b;
    };

    // Build vertex -> face adjacency for union-find.
    std::unordered_map<int, size_t> vertex_to_face;
    vertex_to_face.reserve(its.vertices.size());

    for (size_t fi = 0; fi < n; ++fi) {
        const auto &idx = its.indices[fi];
        for (int vi = 0; vi < 3; ++vi) {
            int v = idx[vi];
            auto it = vertex_to_face.find(v);
            if (it != vertex_to_face.end()) {
                unite(fi, it->second);
            } else {
                vertex_to_face[v] = fi;
            }
        }
    }

    // Count distinct components.
    std::unordered_set<size_t> components;
    for (size_t fi = 0; fi < n; ++fi)
        components.insert(find(fi));

    if (components.size() > 1) {
        result.issues.emplace_back(
            STLValidationIssue::Type::DisconnectedComponent,
            STLValidationIssue::Severity::Warning,
            SIZE_MAX,
            "Mesh has " + std::to_string(components.size()) + " disconnected component(s).");
    }
}

} // namespace Slic3r
