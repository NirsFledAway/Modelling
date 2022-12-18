function sys=mdlDerivatives(t, x, uu, MAV)
    data = do_model_calc(t, x, uu, MAV);

    sys = data.sys;
end
    