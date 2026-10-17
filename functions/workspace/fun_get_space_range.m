function space_range = fun_get_space_range(config);

    space_range = struct;

    if config.use_custom_bound
        space_range.lower = config.custom_workspace_lower_bound;
        space_range.upper = config.custom_workspace_upper_bound;
    else
        space_range.lower = -config.arm_length * ones(1, 3);
        space_range.upper = config.arm_length * ones(1, 3);
    end

end
