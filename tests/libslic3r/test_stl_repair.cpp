#include <catch2/catch_all.hpp>
#include "test_utils.hpp"

#include <libslic3r/TriangleMesh.hpp>
#include <libslic3r/Model.hpp>
#include <libslic3r/STLValidator.hpp>
#include <libslic3r/STLRepair.hpp>
#include <libslic3r/Format/STL.hpp>

using namespace Slic3r;

static inline std::string stl_path(const char* path)
{
	return std::string(TEST_DATA_DIR) + "/test_stl/" + path;
}

// Helper to create a simple valid cube mesh.
static TriangleMesh make_test_cube()
{
    return make_cube(20.0, 20.0, 20.0);
}

// Helper to create a mesh with degenerate faces (zero-area triangles).
static TriangleMesh make_degenerate_mesh()
{
    // Create a cube, then add a degenerate face by duplicating a vertex.
    TriangleMesh mesh = make_test_cube();
    // The cube should have 12 faces. Let's corrupt one face.
    if (mesh.its.indices.size() >= 1) {
        auto &idx = mesh.its.indices[0];
        idx[1] = idx[0];  // Make two vertices the same -> degenerate.
    }
    return mesh;
}

// Helper to create a mesh with open edges (non-watertight).
static TriangleMesh make_open_mesh()
{
    // Create a single triangle (always has open edges).
    std::vector<Vec3f> vertices = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f}
    };
    std::vector<Vec3i32> faces = {{0, 1, 2}};
    return TriangleMesh(vertices, faces);
}

// Helper to create a mesh with invalid face indices.
static TriangleMesh make_invalid_index_mesh()
{
    std::vector<Vec3f> vertices = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f}
    };
    // Index 5 is out of bounds for 3 vertices.
    std::vector<Vec3i32> faces = {{0, 1, 5}};
    return TriangleMesh(vertices, faces);
}

// Helper to create a mesh with NaN vertices.
static TriangleMesh make_nan_vertex_mesh()
{
    std::vector<Vec3f> vertices = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, std::numeric_limits<float>::quiet_NaN(), 0.0f},
        {0.0f, 1.0f, 0.0f}
    };
    std::vector<Vec3i32> faces = {{0, 1, 2}};
    return TriangleMesh(vertices, faces);
}

// Helper to create a mesh with flipped faces (negative volume).
static TriangleMesh make_flipped_mesh()
{
    TriangleMesh mesh = make_test_cube();
    // Flip all faces by swapping vertex order.
    for (auto &idx : mesh.its.indices)
        std::swap(idx[0], idx[1]);
    return mesh;
}

// Helper to create a mesh with disconnected components.
static TriangleMesh make_disconnected_mesh()
{
    std::vector<Vec3f> vertices = {
        // Triangle 1
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        // Triangle 2 (completely separate)
        {10.0f, 10.0f, 10.0f},
        {11.0f, 10.0f, 10.0f},
        {10.0f, 11.0f, 10.0f}
    };
    std::vector<Vec3i32> faces = {{0, 1, 2}, {3, 4, 5}};
    return TriangleMesh(vertices, faces);
}

// --- STLValidationResult tests ---

TEST_CASE("STLValidationResult", "[STLValidation]") {
    SECTION("Empty result is valid") {
        STLValidationResult result;
        REQUIRE(result.valid());
        REQUIRE_FALSE(result.has_errors());
        REQUIRE_FALSE(result.has_critical());
        REQUIRE(result.error_count() == 0);
        REQUIRE(result.warning_count() == 0);
        REQUIRE(result.summary() == "Mesh is valid.");
    }

    SECTION("Error count") {
        STLValidationResult result;
        result.issues.emplace_back(
            STLValidationIssue::Type::OpenEdge,
            STLValidationIssue::Severity::Error,
            SIZE_MAX, "open edge");
        REQUIRE_FALSE(result.valid());
        REQUIRE(result.has_errors());
        REQUIRE(result.error_count() == 1);
        REQUIRE(result.warning_count() == 0);
    }

    SECTION("Warning count") {
        STLValidationResult result;
        result.issues.emplace_back(
            STLValidationIssue::Type::DegenerateFacet,
            STLValidationIssue::Severity::Warning,
            0, "degenerate");
        REQUIRE_FALSE(result.valid());
        REQUIRE_FALSE(result.has_errors());
        REQUIRE(result.error_count() == 0);
        REQUIRE(result.warning_count() == 1);
    }

    SECTION("Critical severity") {
        STLValidationResult result;
        result.issues.emplace_back(
            STLValidationIssue::Type::EmptyMesh,
            STLValidationIssue::Severity::Critical,
            SIZE_MAX, "empty");
        REQUIRE(result.has_critical());
    }
}

// --- STLValidator tests ---

TEST_CASE("STLValidator validates a valid cube", "[STLValidation]") {
    TriangleMesh mesh = make_test_cube();
    STLValidator validator;
    STLValidationResult result = validator.validate(mesh);
    REQUIRE(result.valid());
}

TEST_CASE("STLValidator detects empty mesh", "[STLValidation]") {
    indexed_triangle_set its;
    STLValidator validator;
    STLValidationResult result = validator.validate(its);
    REQUIRE_FALSE(result.valid());
    REQUIRE(result.has_critical());
}

TEST_CASE("STLValidator detects degenerate facets", "[STLValidation]") {
    TriangleMesh mesh = make_degenerate_mesh();
    STLValidator validator;
    STLValidationResult result = validator.validate(mesh);
    REQUIRE_FALSE(result.valid());
    // Should have at least one degenerate facet warning.
    bool found_degenerate = false;
    for (const auto &issue : result.issues)
        if (issue.type == STLValidationIssue::Type::DegenerateFacet)
            found_degenerate = true;
    REQUIRE(found_degenerate);
}

TEST_CASE("STLValidator detects open edges", "[STLValidation]") {
    TriangleMesh mesh = make_open_mesh();
    STLValidator validator;
    STLValidationResult result = validator.validate(mesh);
    REQUIRE_FALSE(result.valid());
    bool found_open = false;
    for (const auto &issue : result.issues)
        if (issue.type == STLValidationIssue::Type::OpenEdge)
            found_open = true;
    REQUIRE(found_open);
}

TEST_CASE("STLValidator detects invalid face indices", "[STLValidation]") {
    TriangleMesh mesh = make_invalid_index_mesh();
    STLValidator validator;
    STLValidationResult result = validator.validate(mesh);
    REQUIRE_FALSE(result.valid());
    bool found_invalid = false;
    for (const auto &issue : result.issues)
        if (issue.type == STLValidationIssue::Type::InvalidFaceIndex)
            found_invalid = true;
    REQUIRE(found_invalid);
}

TEST_CASE("STLValidator detects NaN vertices", "[STLValidation]") {
    TriangleMesh mesh = make_nan_vertex_mesh();
    STLValidator validator;
    STLValidationResult result = validator.validate(mesh);
    REQUIRE_FALSE(result.valid());
    bool found_nan = false;
    for (const auto &issue : result.issues)
        if (issue.type == STLValidationIssue::Type::NaNVertex)
            found_nan = true;
    REQUIRE(found_nan);
}

TEST_CASE("STLValidator detects disconnected components", "[STLValidation]") {
    TriangleMesh mesh = make_disconnected_mesh();
    STLValidator validator;
    STLValidationResult result = validator.validate(mesh);
    REQUIRE_FALSE(result.valid());
    bool found_disconnected = false;
    for (const auto &issue : result.issues)
        if (issue.type == STLValidationIssue::Type::DisconnectedComponent)
            found_disconnected = true;
    REQUIRE(found_disconnected);
}

TEST_CASE("STLValidator options can disable checks", "[STLValidation]") {
    TriangleMesh mesh = make_open_mesh();
    STLValidator::Options opts;
    opts.check_open_edges = false;
    opts.check_manifold_edges = false;
    opts.check_disconnected = false;
    STLValidator validator(opts);
    STLValidationResult result = validator.validate(mesh);
    // With open edge check disabled, should only have disconnect warning.
    bool found_open = false;
    for (const auto &issue : result.issues)
        if (issue.type == STLValidationIssue::Type::OpenEdge)
            found_open = true;
    REQUIRE_FALSE(found_open);
}

// --- STLRepair tests ---

TEST_CASE("STLRepair on a valid cube does nothing", "[STLRepair]") {
    TriangleMesh mesh = make_test_cube();
    STLRepair repair;
    STLRepairResult result = repair.repair(mesh);
    REQUIRE_FALSE(result.anything_repaired());
}

TEST_CASE("STLRepair removes degenerate faces", "[STLRepair]") {
    TriangleMesh mesh = make_degenerate_mesh();
    STLRepair repair;
    STLRepairResult result = repair.repair(mesh);
    REQUIRE(result.degenerate_facets_removed > 0);
    // After repair, mesh should have fewer faces.
    REQUIRE(mesh.its.indices.size() < 13);  // Cube has 12 faces.
}

TEST_CASE("STLRepair removes invalid face indices", "[STLRepair]") {
    TriangleMesh mesh = make_invalid_index_mesh();
    STLRepair repair;
    STLRepairResult result = repair.repair(mesh);
    REQUIRE(result.invalid_faces_removed > 0);
    REQUIRE(mesh.its.indices.empty());  // The only face was invalid.
}

TEST_CASE("STLRepair fixes flipped orientation", "[STLRepair]") {
    TriangleMesh mesh = make_flipped_mesh();
    STLRepair repair;
    STLRepair::Options opts;
    opts.fix_orientation = true;
    repair.set_options(opts);
    STLRepairResult result = repair.repair(mesh);
    REQUIRE(result.orientation_fixed);

    // Verify volume is now positive.
    float vol = its_volume(mesh.its);
    REQUIRE(vol > 0.0f);
}

TEST_CASE("STLRepair merges duplicate vertices", "[STLRepair]") {
    // Create two triangles sharing an edge but with slightly offset vertices.
    std::vector<Vec3f> vertices = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.5f, 1.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},  // Duplicate of vertex 1
        {1.5f, 1.0f, 0.0f},
        {0.5f, 1.0f, 0.0f}   // Duplicate of vertex 2
    };
    std::vector<Vec3i32> faces = {{0, 1, 2}, {3, 4, 5}};
    TriangleMesh mesh(vertices, faces);

    STLRepair repair;
    STLRepairResult result = repair.repair(mesh);
    REQUIRE(result.vertices_merged > 0);
    // After merging, should have fewer unique vertices.
    REQUIRE(mesh.its.vertices.size() < 6);
}

TEST_CASE("STLRepair works on disconnected mesh", "[STLRepair]") {
    TriangleMesh mesh = make_disconnected_mesh();
    STLRepair repair;
    STLRepairResult result = repair.repair(mesh);
    // Should not crash, even if it can't merge the components.
    REQUIRE(mesh.its.indices.size() == 2);
}

TEST_CASE("STLRepair repair_copy preserves original", "[STLRepair]") {
    TriangleMesh original = make_degenerate_mesh();
    TriangleMesh copy = original;
    STLRepair repair;
    TriangleMesh repaired = repair.repair_copy(original);
    // Original should be unchanged.
    REQUIRE(original.its.indices.size() == copy.its.indices.size());
    // Repaired should have fewer faces.
    REQUIRE(repaired.its.indices.size() < original.its.indices.size());
}

// --- Integration with TriangleMesh ---

TEST_CASE("TriangleMesh::validate works", "[STLValidation]") {
    TriangleMesh mesh = make_test_cube();
    STLValidationResult result = mesh.validate();
    REQUIRE(result.valid());
}

TEST_CASE("TriangleMesh::repair_mesh works", "[STLRepair]") {
    TriangleMesh mesh = make_degenerate_mesh();
    STLRepairResult result = mesh.repair_mesh();
    REQUIRE(result.anything_repaired());
}

// --- Load existing test STL files and validate ---

TEST_CASE("Loading and validating test STL files", "[STLValidation]") {
    GIVEN("a valid binary STL file") {
        Model model;
        WHEN("STL file is loaded") {
            REQUIRE(load_stl(stl_path("Geräte/20mmbox-čřšřěá.stl").c_str(), &model));
            THEN("mesh should validate as valid") {
                REQUIRE_FALSE(model.objects.empty());
                STLValidationResult result = model.objects.front()->volumes.front()->mesh().validate();
                REQUIRE(result.valid());
            }
        }
    }

    GIVEN("a valid ASCII STL file") {
        Model model;
        WHEN("STL file is loaded") {
            REQUIRE(load_stl(stl_path("ASCII/20mmbox-LF.stl").c_str(), &model));
            THEN("mesh should validate as valid") {
                REQUIRE_FALSE(model.objects.empty());
                STLValidationResult result = model.objects.front()->volumes.front()->mesh().validate();
                REQUIRE(result.valid());
            }
        }
    }
}
