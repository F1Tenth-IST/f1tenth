#include "mpc/global_to_local.hpp"
#include "mpc/mpc.hpp"

// Define the PointCloud structure for k-d tree
KDTreeWithCloud *prepare_kd_tree(const std::vector<double> &x_traj, const std::vector<double> &y_traj)
{
    PointCloud *cloud = new PointCloud();
    cloud->x_traj = x_traj;
    cloud->y_traj = y_traj;

    my_kd_tree_t *index = new my_kd_tree_t(2 /*dim*/, *cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
    index->buildIndex(); // Build the k-d tree once

    return new KDTreeWithCloud(cloud, index);
}

void MPCNode::global_to_local_pose(my_kd_tree_t *index, const std::vector<double> &s_traj,
                                   const std::vector<double> &x_traj, const std::vector<double> &y_traj,
                                   const std::vector<double> &theta_traj, double x, double y, double psi,
                                   double &s_val, double &n_val, double &u_val, double &X_s, double &Y_s, double &theta_s)
{
    // Query the nearest neighbor (x, y) coordinates using the pre-built k-d tree
    std::vector<double> query_point = {x, y}; // Query point is a 2D point (x, y)
    size_t num_results = 1;                   // Only find the nearest neighbor
    unsigned int idx_query[1];                // Index of the nearest neighbor
    double dists_squared[1];                  // Squared distance of the nearest neighbor

    // Perform the search in the k-d tree (this searches over both x and y)
    index->knnSearch(&query_point[0], num_results, &idx_query[0], &dists_squared[0]);

    size_t closest_idx = idx_query[0]; // Closest index in trajectory

    double track_length = s_traj.back();
    double s_local = s_traj[closest_idx];
    // Detecta wrap para frente (final → início)
    if (s_local < prev_s_local_ - 0.9 * track_length)
    {
        lap_count_ += 1; // completou uma volta
        double lap_time_ms = (this->get_clock()->now() - start_lap_time_).seconds() * 1000.0;
        start_lap_time_ = this->get_clock()->now();

        lap_time_pub_->publish(std_msgs::msg::Float64().set__data(lap_time_ms));
        lap_count_pub_->publish(std_msgs::msg::Int32().set__data(lap_count_));
    }
    // Detecta wrap para trás (início → final)
    else if (s_local > prev_s_local_ + 0.9 * track_length)
    {
        lap_count_ -= 1; // andou uma volta para trás
    }
    // Calcula s acumulativo
    s_val = lap_count_ * track_length + s_local;

    // Atualiza histórico
    prev_s_local_ = s_local;

    X_s = x_traj[closest_idx];
    Y_s = y_traj[closest_idx];
    theta_s = theta_traj[closest_idx]; // Orientation from theta_traj

    // n > 0 para a esquerda da trajetória
    n_val = (y - Y_s) * std::cos(theta_s) - (x - X_s) * std::sin(theta_s);

    // Compute mu_val (heading difference) from the global orientation
    double heading_error = psi - theta_s;
    // Normalize to [-pi, pi]
    u_val = std::remainder(heading_error, 2.0 * M_PI);
}

void local_to_global_pose(
    const std::vector<double> &s_traj,
    const std::vector<double> &x_traj,
    const std::vector<double> &y_traj,
    const std::vector<double> &theta_traj,
    double s_val, double n_val, double u_val,
    double &x, double &y, double &psi,
    double &X_s, double &Y_s, double &theta_s)
{
    // Encontra o índice mais próximo em s_traj
    const double ds = s_traj[1] - s_traj[0];

    size_t closest_idx = static_cast<size_t>(std::round((s_val / ds)));

    if (closest_idx >= s_traj.size())
    {
        closest_idx = closest_idx % s_traj.size();
    }

    // Ponto base na trajetória (coordenadas do caminho)
    X_s = x_traj[closest_idx];
    Y_s = y_traj[closest_idx];
    theta_s = theta_traj[closest_idx];

    // Converte de Frenet (s, n, u) → Global (x, y, ψ)
    // n é o deslocamento lateral (positivo para esquerda da trajetória)
    x = X_s - n_val * std::sin(theta_s);
    y = Y_s + n_val * std::cos(theta_s);

    // ψ (heading global) = heading da trajetória + diferença local
    psi = std::remainder(theta_s + u_val, 2.0 * M_PI);
}

// Função para calcular a integração acumulada de kappa para obter theta
void cumtrapz(const std::vector<double> &s_traj, const std::vector<double> &kappa_traj, std::vector<double> &theta_traj)
{
    // Certifique-se de que as dimensões de s_traj e kappa_traj são iguais
    if (s_traj.size() != kappa_traj.size())
    {
        std::cerr << "Erro: os vetores s_traj e kappa_traj devem ter o mesmo tamanho." << std::endl;
        return;
    }

    // Inicialize o vetor theta_traj com o mesmo tamanho de s_traj
    theta_traj.resize(s_traj.size(), 0.0); // Inicializa com 0 (assumindo theta inicial = 0)

    // Método do trapézio para a integração acumulada
    for (size_t i = 1; i < s_traj.size(); ++i)
    {
        // Calcular a diferença entre os pontos de s_traj
        double ds = s_traj[i] - s_traj[i - 1];

        // Método do trapézio: soma da média dos kappa[i] e kappa[i-1], multiplicado pela diferença ds
        theta_traj[i] = theta_traj[i - 1] + 0.5 * ds * (kappa_traj[i] + kappa_traj[i - 1]);
    }
}

/* int main()
{
    // Example trajectory data (ensure they're properly initialized)
    std::vector<double> s_traj = {0.0, 1.0, 2.0, 3.0};     // Example s_traj
    std::vector<double> x_traj = {0.0, 1.0, 2.0, 3.0};     // Example x_traj
    std::vector<double> y_traj = {0.0, 1.0, 2.0, 3.0};     // Example y_traj
    std::vector<double> theta_traj = {0.0, 0.1, 0.2, 0.3}; // Example theta_traj

    // Prepare the k-d tree
    KDTreeWithCloud *kd_tree = prepare_kd_tree(x_traj, y_traj);

    // Example global coordinates and heading
    double x = 4.1;
    double y = 4.1;
    double psi = 0.15; // Example psi (orientation)

    // Initialize output variables
    double s_val, n_val, u_val, X_s, Y_s, theta_s;

    // Call the global_to_local_pose function
    global_to_local_pose(kd_tree->index, s_traj, x_traj, y_traj, theta_traj, x, y, psi, s_val, n_val, u_val, X_s, Y_s, theta_s);

    // Output the results
    std::cout << "s_val: " << s_val << ", n_val: " << n_val << ", u_val: " << u_val << std::endl;
    std::cout << "X_s: " << X_s << ", Y_s: " << Y_s << ", theta_s: " << theta_s << std::endl;

    // Clean up the allocated k-d tree memory
    delete kd_tree;

    return 0;
} */
