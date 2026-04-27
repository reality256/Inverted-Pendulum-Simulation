clear; clc;

model = "cartpole_pid_simulink";
load_system(model);

Fmax = 50;

% Fine search ranges
Kp_list = 60:5:120;              % step 5
Kd_list = 2:1:22;                % step 1
Kx_list = -0.05:0.002:-0.002;    % step 0.002
Kv_list = -0.25:0.01:-0.02;      % step 0.01
Ki_list = 0:0.02:0.5;            % step 0.02


best_cost = inf;
best_params = [];
results = [];

count = 0;
total = numel(Kp_list)*numel(Kd_list)*numel(Kx_list)*numel(Kv_list)*numel(Ki_list);

for Kp = 120
    for Kd = 5
        for Kx = Kx_list
            for Kv = Kv_list
                for Ki = 0.5

                    count = count + 1;

                    assignin("base", "Kp", Kp);
                    assignin("base", "Kd", Kd);
                    assignin("base", "Kx", Kx);
                    assignin("base", "Kv", Kv);
                    assignin("base", "Ki", Ki);
                    assignin("base", "Fmax", Fmax);

                    try
                        simOut = sim(model, ...
                            "StopTime", "10", ...
                            "ReturnWorkspaceOutputs", "on");

                        theta_ts = simOut.get("theta_log");
                        x_ts     = simOut.get("x_log");
                        u_ts     = simOut.get("u_log");

                        theta = theta_ts.Data;
                        x     = x_ts.Data;
                        u     = u_ts.Data;
                        t     = theta_ts.Time;

                        % Reject unstable cases
                        if any(isnan(theta)) || any(abs(theta) > deg2rad(45)) || any(abs(x) > 10) || any(abs(u) >= Fmax*0.999)
                            cost = inf;
                        else
                            cost_theta = trapz(t, theta.^2);
                            cost_x     = 0.2   * trapz(t, x.^2);
                            cost_u     = 0.001 * trapz(t, u.^2);

                            final_theta_penalty = 200 * theta(end)^2;
                            final_x_penalty     = 5   * x(end)^2;

                            cost = cost_theta + cost_x + cost_u + final_theta_penalty + final_x_penalty;
                        end

                    catch
                        cost = inf;
                    end

                    results = [results; Kp Kd Kx Kv Ki cost];

                    if cost < best_cost
                        best_cost = cost;
                        best_params = [Kp Kd Kx Kv Ki];

                        fprintf("New best: Kp=%.2f, Kd=%.2f, Kx=%.4f, Kv=%.4f, Ki=%.3f, cost=%.6f\n", Kp, Kd, Kx, Kv, Ki, cost);
                    end

                    if mod(count, 500) == 0
                        fprintf("Progress: %d / %d\n", count, total);
                    end

                end
            end
        end
    end
end

results_table = array2table(results, ...
    "VariableNames", ["Kp", "Kd", "Kx", "Kv", "Ki", "Cost"]);

results_table = sortrows(results_table, "Cost", "ascend");

disp("Best parameters:");
disp(array2table(best_params, ...
    "VariableNames", ["Kp", "Kd", "Kx", "Kv", "Ki"]));

disp("Top 20 results:");
disp(results_table(1:min(20,height(results_table)), :));