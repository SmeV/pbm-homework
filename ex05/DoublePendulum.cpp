#include "DoublePendulum.h"

DoublePendulum::DoublePendulum(std::vector<particle_type> &particles, const Length &l1, const Length &l2):
	System(particles), l1(l1), l2(l2) {}

void DoublePendulum::computeAccelerations() {
	/* Read the angle ("position") and angular velocity ("velocity") of both particles from the "particles" vector.
	 * Then compute the angular accelerations and store them in the particles' "acceleration" variable.
	 */
	Angle pos1 = particles[0].position;
	Angle pos2 = particles[1].position;

	AngularVelocity vel1 = particles[0].velocity;
	AngularVelocity vel2 = particles[1].velocity;

	Mass m1 = particles[0].mass;
	Mass m2 = particles[1].mass;

	Acceleration grav1D(9.81 * m / (s*s));

	particles[0].acceleration = (m2*l1*vel1*vel1*sin(pos2-pos1)*cos(pos2-pos1) + m2*grav1D*sin(pos2)*cos(pos2-pos1) + m2*l2*vel2*vel2*sin(pos2-pos1) - (m1 + m2)*grav1D*sin(pos1)) / ((m1+m2)*l2 - m2*l1*cos(pos2-pos1)*cos(pos2-pos1));
	particles[1].acceleration = (-m2*l2*vel2*vel2*sin(pos2-pos1)*cos(pos2-pos1) + (m1+m2)*(grav1D*sin(pos1)*cos(pos2-pos1) - l1*vel2*vel2*sin(pos2-pos1) - grav1D*sin(pos2))) / ((m1+m2)*l2 - m2*l2*cos(pos2-pos1)*cos(pos2-pos1));
}

