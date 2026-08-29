model My_Pendulum
parameter Real Kp = 11.0;   // must exceed M_grav ≈ 1.66 to overcome gravity
parameter Real Kd = 0.9;

Modelica.Blocks.Sources.RealExpression pdLaw(
    y = Kp*(revolute.phi - Modelica.Constants.pi/2) + Kd*revolute.w)annotation(
    Placement(transformation(origin = {47.0112, 34.9507}, extent = {{-26.2622, -28.2852}, {26.2622, 28.2852}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed annotation(
    Placement(transformation(origin = {-10, 66}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute(phi(start = -1.5708, fixed = true, displayUnit = "rad"), w(start = 0, fixed = true))  annotation(
    Placement(transformation(origin = {-10, 34}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder pendulum(r = {0.3, 0, 0}, diameter = 0.017)  annotation(
    Placement(transformation(origin = {-10, 4}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  inner Modelica.Mechanics.MultiBody.World world annotation(
    Placement(transformation(origin = {-68, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Revolute wheelJoint(phi(fixed = true), w(fixed = true), useAxisFlange = true)  annotation(
    Placement(transformation(origin = {-10, -24}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder wheel( diameter = 0.1, r = {0, 0, 0.005})  annotation(
    Placement(transformation(origin = {-10, -54}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.Rotational.Sources.Torque torque(useSupport = false)  annotation(
    Placement(transformation(origin = {22, -24}, extent = {{10, -10}, {-10, 10}})));
equation
  connect(revolute.frame_a, fixed.frame_b) annotation(
    Line(points = {{-10, 44}, {-10, 56}}, color = {95, 95, 95}));
  connect(revolute.frame_b, pendulum.frame_a) annotation(
    Line(points = {{-10, 24}, {-10, 14}}, color = {95, 95, 95}));
  connect(pendulum.frame_b, wheelJoint.frame_a) annotation(
    Line(points = {{-10, -6}, {-10, -14}}, color = {95, 95, 95}));
  connect(wheelJoint.frame_b, wheel.frame_a) annotation(
    Line(points = {{-10, -34}, {-10, -44}}, color = {95, 95, 95}));
  connect(torque.flange, wheelJoint.axis) annotation(
    Line(points = {{12, -24}, {0, -24}}));
  connect(pdLaw.y, torque.tau) annotation(
    Line(points = {{47, 6}, {47.5, 6}, {47.5, -24}, {34, -24}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "4.1.0")),
  experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.02));
end My_Pendulum;
