function tau_out = apply_torque_limits(tau)

tau_max = [150;150;150;28;28;28];

tau = tau(:);   % ensure column vector

scale = max(abs(tau)./tau_max);

if scale > 1
    tau = tau/scale;
end

tau_out = tau;

end