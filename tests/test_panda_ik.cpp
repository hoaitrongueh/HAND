#include "hand/ik.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Vec = Eigen::Vector3d;
using Q = std::array<double, 7>;
using Model = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using Data = std::unique_ptr<mjData, decltype(&mj_deleteData)>;
constexpr std::uint32_t seed = 0x48414e44;
constexpr double tolerance = 0.001;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
Model load(const std::string& path) {
    char error[2048] = {};
    Model m(mj_loadXML(path.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
    require(bool(m), "Model load failed: " + std::string(error));
    return m;
}
struct Robot {
    Model m;
    Data d;
    int body;
    explicit Robot(const std::string& path = HAND_PANDA_MODEL)
        : m(load(path)), d(mj_makeData(m.get()), mj_deleteData),
          body(mj_name2id(m.get(), mjOBJ_BODY, "hand")) {
        require(bool(d) && body >= 0, "Missing data/hand");
        const int home = mj_name2id(m.get(), mjOBJ_KEY, "home");
        if (home >= 0) mj_resetDataKeyframe(m.get(), d.get(), home);
        mj_forward(m.get(), d.get());
    }
    int joint(int i) const {
        const auto name = "joint" + std::to_string(i + 1);
        const int id = mj_name2id(m.get(), mjOBJ_JOINT, name.c_str());
        require(id >= 0, "Fixture missing " + name);
        return id;
    }
    Vec position() const {
        return Eigen::Map<const Vec>(d->xpos + 3 * body);
    }
    Q angles() const {
        Q q{};
        for (int i = 0; i < 7; ++i) q[i] = d->qpos[m->jnt_qposadr[joint(i)]];
        return q;
    }
    void set(const Q& q) {
        for (int i = 0; i < 7; ++i) d->qpos[m->jnt_qposadr[joint(i)]] = q[i];
        mj_forward(m.get(), d.get());
    }
    std::vector<double> full_q() const { return {d->qpos, d->qpos + m->nq}; }
    bool valid() const {
        for (int i = 0; i < 7; ++i) {
            const int id = joint(i);
            const double q = d->qpos[m->jnt_qposadr[id]];
            if (!std::isfinite(q)) return false;
            if (m->jnt_limited[id] &&
                (q < m->jnt_range[2 * id] || q > m->jnt_range[2 * id + 1])) return false;
        }
        return true;
    }
};

template<class T> std::string values(const T& q) {
    std::ostringstream s;
    s << std::setprecision(17);
    for (const auto v : q) s << v << ' ';
    return s.str();
}
std::string values(const Vec& q) { return values(std::array<double, 3>{q.x(), q.y(), q.z()}); }

struct Record {
    std::string category, id;
    std::vector<double> initial, final;
    Vec target;
    Q witness{};
    bool has_witness = false;
    hand::IkResult result;
    double microseconds = 0;
    int increases = 0, stagnant = 0;
    bool net_worse = false;
};
std::vector<Record> records;

Record run(Robot& r, const Vec& target, const std::string& category,
           const std::string& id, const Q* witness = nullptr,
           const hand::IkOptions& options = {}, bool check = true) {
    Record rec;
    rec.category = category; rec.id = id; rec.target = target;
    rec.initial = r.full_q();
    if (witness) { rec.witness = *witness; rec.has_witness = true; }
    const auto start = std::chrono::steady_clock::now();
    rec.result = hand::solve_panda_ik(r.m.get(), r.d.get(), target, "hand", options);
    const auto stop = std::chrono::steady_clock::now();
    rec.microseconds = std::chrono::duration<double, std::micro>(stop - start).count();
    rec.final = r.full_q();
    if (check) mj_forward(r.m.get(), r.d.get());
    for (const auto& s : rec.result.steps) {
        if (s.accepted && s.error_after > s.error_before + 1e-12) ++rec.increases;
        if (s.accepted && s.error_before > tolerance &&
            std::abs(s.error_after - s.error_before) <= 1e-10) ++rec.stagnant;
    }
    rec.net_worse = !rec.result.steps.empty() &&
        rec.result.final_error > rec.result.steps.front().error_before + 1e-12;
    records.push_back(rec);
    std::cout << category << '/' << id << " target=[" << values(target)
              << "] initial=[" << values(rec.initial) << "] final=[" << values(rec.final)
              << "] error=" << rec.result.final_error << " updates=" << rec.result.iterations
              << " reason=" << hand::to_string(rec.result.reason) << '\n';
    if (check) {
        require(r.valid(), "Non-finite or out-of-limit final arm state");
        require(std::isfinite(rec.result.final_error), "Non-finite reported error");
        require(std::abs((target - r.position()).norm() - rec.result.final_error) < 1e-12,
                "Reported error disagrees with independent FK position");
        require(rec.result.iterations <= options.max_iterations, "Unbounded iterations");
        require(!rec.result.success || rec.result.final_error <= options.tolerance,
                "False-positive convergence");
        std::vector<bool> arm(r.m->nq, false);
        for (int i = 0; i < 7; ++i) arm[r.m->jnt_qposadr[r.joint(i)]] = true;
        for (int i = 0; i < r.m->nq; ++i)
            require(arm[i] || rec.initial[i] == rec.final[i], "Non-arm qpos changed");
        for (const auto& s : rec.result.steps)
            require(std::isfinite(s.error_before) && std::isfinite(s.error_after),
                    "Non-finite trace");
    }
    return rec;
}

double percentile(std::vector<double> x, double p) {
    if (x.empty()) return 0;
    std::sort(x.begin(), x.end());
    return x[static_cast<std::size_t>(std::ceil(p * x.size())) - 1];
}
void save(const std::string& name) {
    std::filesystem::create_directories(HAND_IK_OUTPUT);
    std::ofstream csv(std::string(HAND_IK_OUTPUT) + '/' + name + ".csv");
    std::ofstream trace(std::string(HAND_IK_OUTPUT) + '/' + name + "_trace.csv");
    require(bool(csv) && bool(trace), "Cannot create evidence files");
    csv << "seed,category,id,initial_qpos,target_xyz,witness_arm_q,final_qpos,success,error_m,updates,reason,solver_us,increasing_steps,stagnant_steps,net_worse\n";
    trace << "category,id,proposal,error_before_m,error_after_m,accepted\n";
    csv << std::setprecision(17); trace << std::setprecision(17);
    for (const auto& r : records) {
        csv << seed << ',' << r.category << ',' << r.id << ",\"" << values(r.initial)
            << "\",\"" << values(r.target) << "\",\"";
        if (r.has_witness) csv << values(r.witness);
        csv << "\",\"" << values(r.final) << "\"," << r.result.success << ','
            << r.result.final_error << ',' << r.result.iterations << ','
            << hand::to_string(r.result.reason) << ',' << r.microseconds << ','
            << r.increases << ',' << r.stagnant << ',' << r.net_worse << '\n';
        for (std::size_t i = 0; i < r.result.steps.size(); ++i) {
            const auto& s = r.result.steps[i];
            trace << r.category << ',' << r.id << ',' << i << ',' << s.error_before << ','
                  << s.error_after << ',' << s.accepted << '\n';
        }
    }
}

Q sample(Robot& r, std::mt19937& rng, double margin = 0.05) {
    Q q{};
    for (int i = 0; i < 7; ++i) {
        const int id = r.joint(i);
        const double lo = r.m->jnt_range[2 * id], hi = r.m->jnt_range[2 * id + 1];
        const double u = static_cast<double>(rng()) / 4294967296.0;
        q[i] = lo + (hi - lo) * (margin + (1 - 2 * margin) * u);
    }
    return q;
}

void baseline() {
    Robot r;
    const auto rec = run(r, r.position() + Vec(0.02, 0, 0), "baseline", "x20mm");
    const std::array<double, 6> expected{.02, .0104409, .0054479, .00284176, .00148214, .000773014};
    require(rec.result.success && rec.result.iterations == 5, "Baseline failed/five updates changed");
    require(rec.result.steps.size() == 5, "Baseline trace size changed");
    for (std::size_t i = 0; i < 5; ++i)
        require(std::abs(rec.result.steps[i].error_before - expected[i]) < 6e-8,
                "Baseline trace changed at " + std::to_string(i));
    require(std::abs(rec.result.final_error - expected.back()) < 6e-9, "Baseline final error changed");
}
void directions() {
    for (int axis = 0; axis < 3; ++axis) for (int sign : {-1, 1}) {
        Robot r;
        Vec offset = Vec::Zero(); offset[axis] = sign * .02;
        require(run(r, r.position() + offset, "direction", std::to_string(axis) + '_' +
                    std::to_string(sign)).result.success, "Small directional target failed");
    }
}
void within_tolerance() {
    for (double x : {0.0, 0.0005, 0.000999, 0.001001}) {
        Robot r;
        const auto before = r.full_q();
        const auto rec = run(r, r.position() + Vec(x, 0, 0), "tolerance", std::to_string(x));
        require(rec.result.success, "Tolerance target not solved");
        if (x < tolerance) require(before == r.full_q() && rec.result.iterations == 0,
                                   "Unnecessary movement within tolerance");
        else require(rec.result.iterations > 0, "Outside tolerance accepted without update");
    }
}
void displacements() {
    for (double x : {.005, .02, .05, .1, .2, .4, .8}) {
        Robot r;
        run(r, r.position() + Vec(x, 0, 0), "displacement", std::to_string(x));
    }
}
void configurations() {
    Robot r; std::mt19937 rng(seed);
    for (int i = 0; i < 12; ++i) {
        r.set(sample(r, rng, .15));
        run(r, r.position() + Vec(.005, -.005, .005), "configuration", std::to_string(i));
    }
}
void near_limits() {
    Robot r; const Q home = r.angles();
    for (int i = 0; i < 7; ++i) for (int side = 0; side < 2; ++side) {
        Q q = home; const int id = r.joint(i);
        q[i] = r.m->jnt_range[2 * id + side] + (side ? -1 : 1) * 1e-8;
        r.set(q);
        run(r, r.position() + Vec(.005, .005, .005), "near_limit",
            std::to_string(i + 1) + '_' + std::to_string(side));
    }
}

Eigen::Matrix<double, 3, 7> jacobian(Robot& r) {
    std::vector<mjtNum> p(3 * r.m->nv);
    mj_jacBody(r.m.get(), r.d.get(), p.data(), nullptr, r.body);
    Eigen::Matrix<double, 3, 7> j;
    for (int row = 0; row < 3; ++row) for (int i = 0; i < 7; ++i)
        j(row, i) = p[row * r.m->nv + r.m->jnt_dofadr[r.joint(i)]];
    return j;
}
void jacobian_check() {
    Robot r; const Q home = r.angles(); std::mt19937 rng(seed);
    for (int n = 0; n < 12; ++n) {
        Q q = n == 0 ? home : sample(r, rng, .1);
        if (n == 1) q = {0, 0, 0, -.06980001, 0, 0, 0};
        r.set(q);
        const auto analytic = jacobian(r);
        const Eigen::JacobiSVD<Eigen::Matrix<double, 3, 7>> svd(analytic);
        for (double h : {1e-5, 1e-6, 1e-7}) {
            Eigen::Matrix<double, 3, 7> finite;
            for (int i = 0; i < 7; ++i) {
                Q p = q; p[i] += h; r.set(p); const Vec plus = r.position();
                p[i] -= 2 * h; r.set(p); const Vec minus = r.position();
                finite.col(i) = (plus - minus) / (2 * h);
            }
            const double diff = (analytic - finite).cwiseAbs().maxCoeff();
            std::cout << "JACOBIAN q=[" << values(q) << "] h=" << h << " max_abs=" << diff
                      << " singular_values=" << svd.singularValues().transpose() << '\n';
            require(diff < 2e-8, "Analytic/central finite difference Jacobian mismatch");
        }
    }
}

std::string shifted_fixture() {
    std::ifstream file(HAND_PANDA_SOURCE);
    require(bool(file), "Cannot read Panda fixture source");
    std::string xml((std::istreambuf_iterator<char>(file)), {});
    auto replace = [&](const std::string& from, const std::string& to) {
        const auto pos = xml.find(from); require(pos != std::string::npos, "Fixture anchor missing");
        xml.replace(pos, from.size(), to);
    };
    const auto assets = (std::filesystem::path(HAND_PANDA_SOURCE).parent_path() / "assets").generic_string();
    replace("meshdir=\"assets\"", "meshdir=\"" + assets + "\"");
    replace("<worldbody>", "<worldbody><body name=\"unrelated\" pos=\"5 0 0\"><freejoint/>"
            "<geom type=\"sphere\" size=\".01\" mass=\"1\"/></body>");
    replace("qpos=\"0 0 0", "qpos=\"5 0 0 1 0 0 0 0 0 0");
    std::filesystem::create_directories(HAND_IK_OUTPUT);
    const std::string path = std::string(HAND_IK_OUTPUT) + "/shifted_panda.xml";
    std::ofstream out(path); out << xml; out.close();
    return path;
}
void indexing() {
    Robot standard;
    for (int i = 0; i < 7; ++i) {
        const int id = standard.joint(i);
        std::cout << "MAP joint" << i + 1 << " id=" << id << " qpos="
                  << standard.m->jnt_qposadr[id] << " dof=" << standard.m->jnt_dofadr[id] << '\n';
        require(standard.m->jnt_type[id] == mjJNT_HINGE, "Arm joint not a hinge");
        require(standard.m->jnt_qposadr[id] == i && standard.m->jnt_dofadr[id] == i,
                "Standard Panda arm address mapping changed");
    }
    Robot shifted(shifted_fixture());
    for (int i = 0; i < 7; ++i) {
        const int id = shifted.joint(i);
        require(shifted.m->jnt_qposadr[id] == 7 + i && shifted.m->jnt_dofadr[id] == 6 + i,
                "Shift fixture did not separate qpos/dof addresses");
    }
    shifted.d->qpos[14] = .012; shifted.d->qpos[15] = .031;
    mj_forward(shifted.m.get(), shifted.d.get());
    const auto rec = run(shifted, shifted.position() + Vec(.02, 0, 0), "indexing", "free_sibling");
    require(rec.result.success && rec.result.iterations == 5, "Mapped baseline failed");
    jacobian(shifted);
}

void atomic_rejection() {
    Robot r; const Q home = r.angles();
    const int id = r.joint(5);
    r.m->jnt_range[2 * id] = home[5] - 1e-12;
    r.m->jnt_range[2 * id + 1] = home[5] + 1e-12;
    const auto before = r.full_q();
    const auto rec = run(r, r.position() + Vec(.02, 0, 0), "safety", "atomic_rejection");
    require(rec.result.reason == hand::IkTermination::joint_limit &&
            rec.result.iterations == 0 && rec.result.steps.size() == 1 &&
            !rec.result.steps[0].accepted && r.full_q() == before,
            "Rejected proposal partially committed or retried");
    Robot later; const Q start = later.angles();
    const Q witness{-.60040731942799397, 1.1038278355282545, -.21411064918577694,
        -2.7778767300428822, -1.3713232638383797, 1.2919959273124113, -1.2970355714879649};
    later.set(witness); const Vec target = later.position(); later.set(start);
    hand::IkOptions one; one.max_iterations = 1;
    const auto accepted = run(later, target, "safety", "accepted_prefix", &witness, one);
    require(accepted.result.iterations == 1, "Late-rejection fixture did not first commit an update");
    const auto prefix = later.full_q();
    const auto rejected = run(later, target, "safety", "rejection_after_prefix", &witness, one);
    require(rejected.result.reason == hand::IkTermination::joint_limit &&
            rejected.result.iterations == 0 && later.full_q() == prefix,
            "Rejection after earlier progress modified a partial joint state");
}
void disabled_limits() {
    Robot r; const Q home = r.angles();
    for (int i = 0; i < 7; ++i) {
        const int id = r.joint(i); r.m->jnt_limited[id] = 0;
        r.m->jnt_range[2 * id] = home[i]; r.m->jnt_range[2 * id + 1] = home[i];
    }
    require(run(r, r.position() + Vec(.02, 0, 0), "limits", "disabled").result.success,
            "Disabled limits incorrectly enforced");
}
void invalid_model() {
    Robot r; const auto q = r.full_q(); const Vec t = r.position();
    for (const char* name : {"absent", static_cast<const char*>(nullptr)}) {
        const auto s = hand::solve_panda_ik(r.m.get(), r.d.get(), t, name);
        require(!s.success && s.reason == hand::IkTermination::invalid_model && q == r.full_q(),
                "Missing body not rejected atomically");
    }
    require(hand::solve_panda_ik(nullptr, r.d.get(), t).reason == hand::IkTermination::invalid_model,
            "Null model not rejected");
    require(hand::solve_panda_ik(r.m.get(), nullptr, t).reason == hand::IkTermination::invalid_model,
            "Null data not rejected");
}
void missing_joint(bool at_target) {
    Robot r;
    const int id = r.joint(6); r.m->names[r.m->name_jntadr[id]] = 'X';
    const auto q = r.full_q();
    const auto result = hand::solve_panda_ik(r.m.get(), r.d.get(), r.position() +
                                           Vec(at_target ? 0 : .02, 0, 0));
    std::cout << "missing joint7 at_target=" << at_target << " success=" << result.success
              << " reason=" << hand::to_string(result.reason) << '\n';
    require(!result.success && result.reason == hand::IkTermination::missing_joint && q == r.full_q(),
            "Malformed arm accepted or state modified");
}
void unreachable() {
    Robot r;
    double bound = 0;
    for (int b = r.body; b > 0; b = r.m->body_parentid[b])
        bound += Eigen::Map<const Vec>(r.m->body_pos + 3 * b).norm();
    std::cout << "Conservative root-to-hand reach bound=" << bound << " m\n";
    require(bound < 2, "Unreachable proof no longer applies to model");
    const auto result = run(r, Vec(10, 10, 10), "unreachable", "geometric_bound").result;
    require(!result.success, "False convergence on provably unreachable target");
}
void constrained_target() {
    Robot r; const Q q = r.angles(); const Vec p = r.position();
    for (int i = 0; i < 7; ++i) {
        const int id = r.joint(i);
        r.m->jnt_range[2 * id] = q[i]; r.m->jnt_range[2 * id + 1] = q[i];
    }
    const auto rec = run(r, p + Vec(.02, 0, 0), "constrained", "locked_arm");
    require(!rec.result.success && rec.result.reason == hand::IkTermination::joint_limit,
            "Locked-arm unreachable target not rejected");
}
void singular() {
    Robot r;
    r.set({0, 0, 0, -.06980001, 0, 0, 0});
    const auto j = jacobian(r);
    const Eigen::JacobiSVD<Eigen::Matrix<double, 3, 7>> svd(j);
    std::cout << "Near-straight Panda singular values=" << svd.singularValues().transpose() << '\n';
    run(r, r.position() + Vec(.02, 0, 0), "singular", "near_straight_panda");
    std::mt19937 rng(seed);
    Q weakest{};
    double best_ratio = 1;
    Vec weakest_values = Vec::Zero();
    for (int n = 0; n < 2048; ++n) {
        const Q candidate = sample(r, rng, .001); r.set(candidate);
        const Eigen::JacobiSVD<Eigen::Matrix<double, 3, 7>> candidate_svd(jacobian(r));
        const Vec s = candidate_svd.singularValues();
        if (s[2] / s[0] < best_ratio) {
            best_ratio = s[2] / s[0]; weakest = candidate; weakest_values = s;
        }
    }
    std::cout << "Poorly conditioned Panda seed=" << seed << " samples=2048 q=["
              << values(weakest) << "] singular_values=" << weakest_values.transpose()
              << " condition=" << 1 / best_ratio << '\n';
    require(best_ratio < .02, "Conditioning scan did not find a difficult Panda configuration");
    r.set(weakest); const auto weak_j = jacobian(r);
    Eigen::Matrix<double, 3, 7> finite;
    for (int i = 0; i < 7; ++i) {
        Q perturbed = weakest; perturbed[i] += 1e-6; r.set(perturbed); const Vec plus = r.position();
        perturbed[i] -= 2e-6; r.set(perturbed);
        finite.col(i) = (plus - r.position()) / 2e-6;
    }
    const double difference = (weak_j - finite).cwiseAbs().maxCoeff();
    std::cout << "Poorly conditioned Jacobian finite-difference max_abs=" << difference << '\n';
    require(difference < 2e-8, "Poorly conditioned Panda Jacobian mismatch");
    int strongest = 0;
    for (int i = 1; i < 7; ++i)
        if (weak_j.col(i).norm() > weak_j.col(strongest).norm()) strongest = i;
    const int joint_id = r.joint(strongest);
    const double mid = (r.m->jnt_range[2 * joint_id] + r.m->jnt_range[2 * joint_id + 1]) / 2;
    Q witness = weakest; witness[strongest] += weakest[strongest] < mid ? .02 : -.02;
    r.set(witness); require(r.valid(), "Poor-conditioning witness not valid");
    const Vec reachable = r.position(); r.set(weakest);
    run(r, reachable, "singular", "poorly_conditioned_panda", &witness);
    std::string xml = "<mujoco><worldbody>";
    for (int i = 1; i <= 7; ++i)
        xml += "<body pos=\"0 0 .1\"><joint name=\"joint" + std::to_string(i) +
            "\" axis=\"0 0 1\"/><geom type=\"sphere\" size=\".01\" mass=\"1\"/>";
    xml += "<body name=\"hand\" pos=\"0 0 .1\"/>";
    for (int i = 0; i < 7; ++i) xml += "</body>";
    xml += "</worldbody></mujoco>";
    const std::string path = std::string(HAND_IK_OUTPUT) + "/singular.xml";
    std::ofstream file(path); file << xml; file.close();
    Robot s(path); const auto q = s.full_q();
    require(jacobian(s).norm() == 0, "Fixture is not rank zero");
    const auto rec = run(s, s.position() + Vec(.02, 0, 0), "singular", "rank_zero_fixture");
    require(rec.result.reason == hand::IkTermination::iteration_limit &&
            rec.result.iterations == 100 && rec.stagnant == 100 && q == s.full_q(),
            "Rank-zero damping/bounded stagnation changed");
}
void last_update() {
    Robot r; hand::IkOptions options;
    options.max_iterations = 5;
    const auto rec = run(r, r.position() + Vec(.02, 0, 0), "defect", "last_update", nullptr, options);
    require(rec.result.final_error <= tolerance, "Reproducer did not cross tolerance");
    require(rec.result.success, "Final update reached tolerance but solver reported iteration_limit");
}
void nonfinite(const std::string& name) {
    Robot r; Vec target = r.position(); const auto before = r.full_q();
    const int axis = name.back() == 'x' ? 0 : name.back() == 'y' ? 1 : 2;
    target[axis] = name.substr(0, 3) == "nan" ? std::numeric_limits<double>::quiet_NaN() :
        (name.rfind("neginf", 0) == 0 ? -1 : 1) * std::numeric_limits<double>::infinity();
    const auto rec = run(r, target, "invalid_target", name, nullptr, {}, false);
    require(!rec.result.success, "Non-finite target falsely converged");
    require(r.full_q() == before, "Non-finite target corrupted qpos (NaN bypasses range comparisons)");
}
void nonfinite_state() {
    Robot r; const Vec target = r.position() + Vec(.02, 0, 0);
    r.d->qpos[0] = std::numeric_limits<double>::quiet_NaN();
    mj_forward(r.m.get(), r.d.get());
    const auto rec = run(r, target, "invalid_state", "nan_qpos", nullptr, {}, false);
    require(!rec.result.success, "Non-finite state falsely converged");
    require(rec.result.iterations == 0, "Non-finite state accepted updates instead of rejecting input");
}

void invalid_state_limits() {
    Robot r; Q q = r.angles();
    q[3] = 0; r.set(q);
    require(!r.valid(), "Invalid-state reproducer is no longer out of limits");
    const auto rec = run(r, r.position(), "invalid_state", "joint4_outside_limit", nullptr, {}, false);
    require(!rec.result.success, "Out-of-limit arm configuration reported as converged");
}

void reachable_joint_limit() {
    Robot r; const Q home = r.angles();
    const Q witness{-.60040731942799397, 1.1038278355282545, -.21411064918577694,
        -2.7778767300428822, -1.3713232638383797, 1.2919959273124113, -1.2970355714879649};
    r.set(witness); require(r.valid(), "Reachability witness violates model limits");
    const Vec target = r.position(); r.set(home);
    const auto rec = run(r, target, "known_reachable_regression", "stress_seed_case_2", &witness);
    require(rec.result.success, "FK-witnessed reachable target stopped on joint-limit proposal");
}

void stress() {
    Robot r; const Q home = r.angles(); std::mt19937 rng(seed);
    std::cout << "STRESS seed=" << seed << " (0x48414e44), fixed counts; no time thresholds\n";
    for (int n = 0; n < 120; ++n) {
        const Q witness = sample(r, rng);
        r.set(witness); const Vec target = r.position();
        const Q initial = n % 2 == 0 ? home : sample(r, rng);
        r.set(initial);
        run(r, target, "known_reachable", std::to_string(n), &witness);
    }
    for (int n = 0; n < 60; ++n) {
        const Q initial = sample(r, rng); r.set(initial);
        Vec target;
        for (int axis = 0; axis < 3; ++axis)
            target[axis] = -1.25 + 2.5 * (static_cast<double>(rng()) / 4294967296.0);
        run(r, target, "arbitrary_cartesian", std::to_string(n));
    }
    for (const auto& category : {"known_reachable", "arbitrary_cartesian"}) {
        int count = 0, success = 0, limits = 0, exhaustion = 0, increasing = 0, stagnant = 0, worse = 0;
        std::vector<double> iterations, successful_iterations, times, errors;
        for (const auto& rec : records) if (rec.category == category) {
            ++count; success += rec.result.success;
            limits += rec.result.reason == hand::IkTermination::joint_limit;
            exhaustion += rec.result.reason == hand::IkTermination::iteration_limit;
            increasing += rec.increases > 0; stagnant += rec.stagnant > 0;
            worse += rec.net_worse;
            iterations.push_back(rec.result.iterations); times.push_back(rec.microseconds);
            errors.push_back(rec.result.final_error);
            if (rec.result.success) successful_iterations.push_back(rec.result.iterations);
        }
        std::cout << "BENCHMARK " << category << " n=" << count << " successes=" << success
                  << " limit_rejections=" << limits << " iteration_limit=" << exhaustion
                  << " cases_with_increase=" << increasing << " cases_with_stagnation=" << stagnant
                  << " cases_net_worse=" << worse
                  << " updates_median=" << percentile(iterations, .5)
                  << " updates_p95=" << percentile(iterations, .95)
                  << " success_updates_median=" << percentile(successful_iterations, .5)
                  << " success_updates_p95=" << percentile(successful_iterations, .95)
                  << " us_median=" << percentile(times, .5) << " us_p95=" << percentile(times, .95)
                  << " error_median=" << percentile(errors, .5) << " error_p95=" << percentile(errors, .95)
                  << '\n';
    }
}
}

int main(int argc, char** argv) {
    std::cout << std::setprecision(17);
    if (argc != 2) { std::cerr << "Usage: test_panda_ik <registered-case>\n"; return 2; }
    const std::string name = argv[1];
    std::filesystem::create_directories(HAND_IK_OUTPUT);
    std::cout << "MuJoCo " << mj_versionString() << " Eigen " << EIGEN_VERSION_STRING << " model=" << HAND_PANDA_MODEL
              << " defaults=lambda .05 alpha .5 tol .001 max 100\n";
    int code = 0;
    try {
        if (name == "baseline") baseline();
        else if (name == "directions") directions();
        else if (name == "tolerance") within_tolerance();
        else if (name == "displacements") displacements();
        else if (name == "configurations") configurations();
        else if (name == "near_limits") near_limits();
        else if (name == "jacobian") jacobian_check();
        else if (name == "indexing") indexing();
        else if (name == "atomic_rejection") atomic_rejection();
        else if (name == "disabled_limits") disabled_limits();
        else if (name == "invalid_model") invalid_model();
        else if (name == "missing_joint") missing_joint(false);
        else if (name == "missing_joint_at_target") missing_joint(true);
        else if (name == "unreachable") unreachable();
        else if (name == "constrained_target") constrained_target();
        else if (name == "singular") singular();
        else if (name == "last_update") last_update();
        else if (name == "nonfinite_state") nonfinite_state();
        else if (name == "invalid_state_limits") invalid_state_limits();
        else if (name == "reachable_joint_limit") reachable_joint_limit();
        else if (name.rfind("nan_", 0) == 0 || name.rfind("inf_", 0) == 0 ||
                 name.rfind("neginf_", 0) == 0) nonfinite(name);
        else if (name == "stress") stress();
        else throw std::runtime_error("Unknown test case: " + name);
        std::cout << "PASS " << name << '\n';
    } catch (const std::exception& e) {
        std::cerr << "FAIL " << name << ": " << e.what() << '\n'; code = 1;
    }
    try { save(name); }
    catch (const std::exception& e) { std::cerr << "Evidence save failed: " << e.what() << '\n'; return 2; }
    return code;
}
