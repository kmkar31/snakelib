def lateral_undulation(self, t=0, params=None):

    params = {} if params is None else params

    self.current_gait = "lateral_undulation"

    # Update the current parameters if params is not empty
    self.current_gait_params = self.update_params(
        self.default_gait_params.get(self.current_gait), params
    )

    # Math formulation of lateral undulation is identical to sidewinding
    return self.compound_serpenoid(t, self.current_gait_params)
