/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, SRI International
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \Author: Acorn Pooley */

/* convert quaternion to YPR and other formats. */

#include <tf/tf.h>
#include <boost/math/constants/constants.hpp>

static const double PI = (boost::math::constants::pi<double>());

int g_prec = 10;
int g_width = 0;

bool g_debug = true;

static void canonicalize(tf::Quaternion& q)
{
  q.normalize();
  if (q.w() < 0.0)
    q *= -1.0;
}

static void MatrixToYPR(
  const tf::Matrix3x3& m,
  double *yaw,
  double *pitch,
  double *roll,
  int solution=0)
{
#if 0
printf("  M\n");
printf("  %10.4f  %10.4f %10.4f\n",
m[0][0],
m[0][1],
m[0][2]);
printf("  %10.4f  %10.4f %10.4f\n",
m[1][0],
m[1][1],
m[1][2]);
printf("  %10.4f  %10.4f %10.4f\n",
m[2][0],
m[2][1],
m[2][2]);
#endif
  double x,y,z = 0;
  if (std::abs(m[2][0]) < 1.0)
  {
    y = -asin(m[2][0]);
    if (solution)
      y = PI - y;
    double cy = cos(y);
    x = atan2(m[2][1]/cy, m[2][2]/cy);
    z = atan2(m[1][0]/cy, m[0][0]/cy);
printf("~~~~~ case 1\n");
  }

#if 0
  else
  {
    x = atan2(m[2][1], m[2][2]);
printf("x = atan2(m[2][1], m[2][2]) = atan2(%f , %f) = %f\n",
m[2][1], m[2][2], x);
    if (m[2][0] < 0)
    {
      y = PI * 0.5;
printf("~~~~~ case 2\n");
    }
    else
    {
      y = -PI * 0.5;
printf("~~~~~ case 3\n");
    }
  }

#else
  else if (m[2][0] < 0.0)
  {
    y = PI * 0.5;
    x = atan2(m[0][1], m[0][2]);
printf("~~~~~ case 2      WARNING tf::getEulerYPR BROKEN FOR THIS CASE\n");
printf("x = atan2(m[0][1], m[0][2]) = atan2(%f , %f) = %f\n",
  m[0][1], m[0][2], x);

  }
  else
  {
    y = -PI * 0.5;
    x = atan2(-m[0][1], -m[0][2]);
printf("~~~~~ case 3      WARNING tf::getEulerYPR BROKEN FOR THIS CASE\n");
printf("x = atan2(-m[0][1], -m[0][2]) = atan2(%f , %f) = %f\n",
  -m[0][1], -m[0][2], x);
  }
#endif
    
  *roll = x;    // psi
  *pitch = y;   // theta
  *yaw = z;     // phi
}
// .5 .5 .5 -.5

static void show(tf::Quaternion q)
{
  printf("Quat: qx=%*.*f qy=%*.*f qz=%*.*f qw=%*.*f\n",
    g_width, g_prec, q.x(),
    g_width, g_prec, q.y(),
    g_width, g_prec, q.z(),
    g_width, g_prec, q.w());
  canonicalize(q);
  printf("Quat: qx=%*.*f qy=%*.*f qz=%*.*f qw=%*.*f (normalized)\n",
    g_width, g_prec, q.x(),
    g_width, g_prec, q.y(),
    g_width, g_prec, q.z(),
    g_width, g_prec, q.w());

  double angle = q.getAngle();
  tf::Vector3 axis = q.getAxis();
  {
    if (angle > PI)
    {
      angle -= PI * 2.0;
    }
    double biggest = 0.0;
    for (int i = 0 ; i < 3 ; i++)
    {
      if (std::abs(axis[i]) > std::abs(biggest))
        biggest = axis[i];
    }
    if (biggest < 0.0)
    {
      axis *= -1.0;
      angle *= -1.0;
    }
    tf::Quaternion q1(axis, angle);
    tf::Quaternion q2(axis, -angle);
    if (q2.angleShortestPath(q) < q1.angleShortestPath(q))
    {
      angle *= -1.0;
      printf("WARNING:\n");
      printf("WARNING: q.getAngle() returned negative angle for q.getAxis()!!!\n");
      printf("WARNING:\n");
    }
    printf("Axis: ax=%*.*f ay=%*.*f az=%*.*f angle=%*.*f = %*.*f deg\n",
      g_width, g_prec, axis.getX(),
      g_width, g_prec, axis.getY(),
      g_width, g_prec, axis.getZ(),
      g_width, g_prec, angle,
      g_width, g_prec, angle * 180.0 / PI);
  }

  {
    double angle = q.getAngleShortestPath();
    tf::Vector3 axis = q.getAxis();
    if (angle > PI)
    {
      angle -= PI * 2.0;
    }
    double biggest = 0.0;
    for (int i = 0 ; i < 3 ; i++)
    {
      if (std::abs(axis[i]) > std::abs(biggest))
        biggest = axis[i];
    }
    if (biggest < 0.0)
    {
      axis *= -1.0;
      angle *= -1.0;
    }
    tf::Quaternion q1(axis, angle);
    tf::Quaternion q2(axis, -angle);
    if (q2.angleShortestPath(q) < q1.angleShortestPath(q))
    {
      angle *= -1.0;
    }

    printf("Axis: ax=%*.*f ay=%*.*f az=%*.*f angle=%*.*f = %*.*f deg (shortest)\n",
      g_width, g_prec, axis.getX(),
      g_width, g_prec, axis.getY(),
      g_width, g_prec, axis.getZ(),
      g_width, g_prec, angle,
      g_width, g_prec, angle * 180.0 / PI);
  }

  tf::Matrix3x3 m(q);
  printf("Mtx:       %*.*f   %*.*f   %*.*f\n",
    g_width, g_prec, m[0][0],
    g_width, g_prec, m[0][1],
    g_width, g_prec, m[0][2]);
  printf("           %*.*f   %*.*f   %*.*f\n",
    g_width, g_prec, m[1][0],
    g_width, g_prec, m[1][1],
    g_width, g_prec, m[1][2]);
  printf("           %*.*f   %*.*f   %*.*f\n",
    g_width, g_prec, m[2][0],
    g_width, g_prec, m[2][1],
    g_width, g_prec, m[2][2]);

  double yaw_z1, pitch_y1, roll_x1;
  double yaw_z2, pitch_y2, roll_x2;
#if 0
  m.getEulerYPR(yaw_z1, pitch_y1, roll_x1, 1);
  m.getEulerYPR(yaw_z2, pitch_y2, roll_x2, 2);
#else
  MatrixToYPR(m, &yaw_z1, &pitch_y1, &roll_x1, 0);
  MatrixToYPR(m, &yaw_z2, &pitch_y2, &roll_x2, 1);
#endif

#if 0
  double yaw_z1b, pitch_y1b, roll_x1b;
  double yaw_z2b, pitch_y2b, roll_x2b;
  MatrixToYPR(m, &yaw_z1b, &pitch_y1b, &roll_x1b, 0);
  MatrixToYPR(m, &yaw_z2b, &pitch_y2b, &roll_x2b, 1);
#endif


  printf("URDF: rpy=\"%*.*f %*.*f %*.*f\"\n",
    g_width, g_prec, roll_x1,
    g_width, g_prec, pitch_y1,
    g_width, g_prec, yaw_z1);
  printf("StaticTransformPub: 0 0 0 %*.*f %*.*f %*.*f  (YPR)\n",
    g_width, g_prec, yaw_z1,
    g_width, g_prec, pitch_y1,
    g_width, g_prec, roll_x1);
  printf("YPR:  yaw_z=%*.*f pitch_y=%*.*f roll_x=%*.*f  (solution 1)\n",
    g_width, g_prec, yaw_z1,
    g_width, g_prec, pitch_y1,
    g_width, g_prec, roll_x1);
  printf("YPR:  yaw_z=%*.*f pitch_y=%*.*f roll_x=%*.*f  (solution 2)\n",
    g_width, g_prec, yaw_z2,
    g_width, g_prec, pitch_y2,
    g_width, g_prec, roll_x2);

  printf("YPR:  yaw_z=%*.*f pitch_y=%*.*f roll_x=%*.*f  (s1 degrees)\n",
    g_width, g_prec, yaw_z1 * 180.0/PI,
    g_width, g_prec, pitch_y1 * 180.0/PI,
    g_width, g_prec, roll_x1 * 180.0/PI);
  printf("YPR:  yaw_z=%*.*f pitch_y=%*.*f roll_x=%*.*f  (s2 degrees)\n",
    g_width, g_prec, yaw_z2 * 180.0/PI,
    g_width, g_prec, pitch_y2 * 180.0/PI,
    g_width, g_prec, roll_x2 * 180.0/PI);

#if 0
  printf("YPRb: yaw_z=%*.*f pitch_y=%*.*f roll_x=%*.*f  (s1 degrees)\n",
    g_width, g_prec, yaw_z1b * 180.0/PI,
    g_width, g_prec, pitch_y1b * 180.0/PI,
    g_width, g_prec, roll_x1b * 180.0/PI);
  printf("YPRb: yaw_z=%*.*f pitch_y=%*.*f roll_x=%*.*f  (s2 degrees)\n",
    g_width, g_prec, yaw_z2b * 180.0/PI,
    g_width, g_prec, pitch_y2b * 180.0/PI,
    g_width, g_prec, roll_x2b * 180.0/PI);

#endif

  printf(" ignore below here----\n");

  double e_yaw_z1, e_pitch_y1, e_roll_x1;
  m.getEulerYPR(e_yaw_z1, e_pitch_y1, e_roll_x1, 1);
  printf("EYPR: yaw_z=%*.*f pitch_y=%*.*f roll_x=%*.*f  (solution 1)\n",
    g_width, g_prec, e_yaw_z1,
    g_width, g_prec, e_pitch_y1,
    g_width, g_prec, e_roll_x1);
  double e_yaw_z2, e_pitch_y2, e_roll_x2;
  m.getEulerYPR(e_yaw_z2, e_pitch_y2, e_roll_x2, 2);
  printf("EYPR: yaw_z=%*.*f pitch_y=%*.*f roll_x=%*.*f  (solution 2)\n",
    g_width, g_prec, e_yaw_z2,
    g_width, g_prec, e_pitch_y2,
    g_width, g_prec, e_roll_x2);

  printf("EYPR: yaw_z=%*.*f pitch_y=%*.*f roll_x=%*.*f  (s1 degrees)\n",
    g_width, g_prec, e_yaw_z1 * 180.0/PI,
    g_width, g_prec, e_pitch_y1 * 180.0/PI,
    g_width, g_prec, e_roll_x1 * 180.0/PI);
  printf("EYPR: yaw_z=%*.*f pitch_y=%*.*f roll_x=%*.*f  (s2 degrees)\n",
    g_width, g_prec, e_yaw_z2 * 180.0/PI,
    g_width, g_prec, e_pitch_y2 * 180.0/PI,
    g_width, g_prec, e_roll_x2 * 180.0/PI);








#if 1
  printf("Quat: qx=%*.*f qy=%*.*f qz=%*.*f qw=%*.*f (normalized)\n",
    g_width, g_prec, q.x(),
    g_width, g_prec, q.y(),
    g_width, g_prec, q.z(),
    g_width, g_prec, q.w());

  tf::Quaternion mq;
  m.getRotation(mq);
  canonicalize(mq);
  printf("M->Q: qx=%*.*f qy=%*.*f qz=%*.*f qw=%*.*f (normalized)\n",
    g_width, g_prec, mq.x(),
    g_width, g_prec, mq.y(),
    g_width, g_prec, mq.z(),
    g_width, g_prec, mq.w());

    tf::Quaternion aaq(axis, angle);
  canonicalize(aaq);
  printf("A->Q: qx=%*.*f qy=%*.*f qz=%*.*f qw=%*.*f (normalized)\n",
    g_width, g_prec, aaq.x(),
    g_width, g_prec, aaq.y(),
    g_width, g_prec, aaq.z(),
    g_width, g_prec, aaq.w());

  tf::Quaternion rpyq;
  rpyq.setRPY(roll_x1, pitch_y1, yaw_z1);
  canonicalize(rpyq);
  printf("R->Q: qx=%*.*f qy=%*.*f qz=%*.*f qw=%*.*f (normalized)\n",
    g_width, g_prec, rpyq.x(),
    g_width, g_prec, rpyq.y(),
    g_width, g_prec, rpyq.z(),
    g_width, g_prec, rpyq.w());

  tf::Quaternion rpyq2;
  rpyq2.setRPY(roll_x2, pitch_y2, yaw_z2);
  canonicalize(rpyq2);
  printf("2->Q: qx=%*.*f qy=%*.*f qz=%*.*f qw=%*.*f (normalized)\n",
    g_width, g_prec, rpyq2.x(),
    g_width, g_prec, rpyq2.y(),
    g_width, g_prec, rpyq2.z(),
    g_width, g_prec, rpyq2.w());




#if 0
  tf::Quaternion rpyqb;
  rpyqb.setRPY(roll_x1b, pitch_y1b, yaw_z1b);
  canonicalize(rpyqb);
  printf("R->Qb qx=%*.*f qy=%*.*f qz=%*.*f qw=%*.*f (normalized)\n",
    g_width, g_prec, rpyqb.x(),
    g_width, g_prec, rpyqb.y(),
    g_width, g_prec, rpyqb.z(),
    g_width, g_prec, rpyqb.w());

  tf::Quaternion rpyq2b;
  rpyq2b.setRPY(roll_x2b, pitch_y2b, yaw_z2b);
  canonicalize(rpyq2b);
  printf("2->Qb qx=%*.*f qy=%*.*f qz=%*.*f qw=%*.*f (normalized)\n",
    g_width, g_prec, rpyq2b.x(),
    g_width, g_prec, rpyq2b.y(),
    g_width, g_prec, rpyq2b.z(),
    g_width, g_prec, rpyq2b.w());
#endif







  tf::Quaternion e_rpyq;
  e_rpyq.setEuler(e_yaw_z1, e_pitch_y1, e_roll_x1);
  canonicalize(e_rpyq);
  printf("E->Q: qx=%*.*f qy=%*.*f qz=%*.*f qw=%*.*f (normalized)\n",
    g_width, g_prec, e_rpyq.x(),
    g_width, g_prec, e_rpyq.y(),
    g_width, g_prec, e_rpyq.z(),
    g_width, g_prec, e_rpyq.w());

  tf::Quaternion e_rpyq2;
  e_rpyq2.setEuler(e_yaw_z2, e_pitch_y2, e_roll_x2);
  canonicalize(e_rpyq2);
  printf("2->Q: qx=%*.*f qy=%*.*f qz=%*.*f qw=%*.*f (normalized)\n",
    g_width, g_prec, e_rpyq2.x(),
    g_width, g_prec, e_rpyq2.y(),
    g_width, g_prec, e_rpyq2.z(),
    g_width, g_prec, e_rpyq2.w());





  tf::Quaternion e_rpyqb;
  e_rpyqb.setEuler(e_roll_x1, e_pitch_y1, e_yaw_z1);
  canonicalize(e_rpyqb);
  printf("E>>Q: qx=%*.*f qy=%*.*f qz=%*.*f qw=%*.*f (normalized)\n",
    g_width, g_prec, e_rpyqb.x(),
    g_width, g_prec, e_rpyqb.y(),
    g_width, g_prec, e_rpyqb.z(),
    g_width, g_prec, e_rpyqb.w());

  tf::Quaternion e_rpyq2b;
  e_rpyq2b.setEuler(e_roll_x2, e_pitch_y2, e_yaw_z2);
  canonicalize(e_rpyq2b);
  printf("2>>Q: qx=%*.*f qy=%*.*f qz=%*.*f qw=%*.*f (normalized)\n",
    g_width, g_prec, e_rpyq2b.x(),
    g_width, g_prec, e_rpyq2b.y(),
    g_width, g_prec, e_rpyq2b.z(),
    g_width, g_prec, e_rpyq2b.w());


  tf::Matrix3x3 rpym;
  rpym.setEulerYPR(e_roll_x2, e_pitch_y2, e_yaw_z2);
  tf::Quaternion rpymq;
  rpym.getRotation(rpymq);
  printf("RMQ:  qx=%*.*f qy=%*.*f qz=%*.*f qw=%*.*f (normalized)\n",
    g_width, g_prec, rpymq.x(),
    g_width, g_prec, rpymq.y(),
    g_width, g_prec, rpymq.z(),
    g_width, g_prec, rpymq.w());



// .5 .5 .5 -.5



#endif
}


bool readRot(const char **ss, tf::Quaternion *result)
{
  const char *line = *ss;
  const char *s = line;

  while (*s == ' ')
    s++;

  bool use_rpy = false;
  bool use_aa = false;
  if (s[0] == 'r' && s[1] == 'p' && s[2] == 'y')
    use_rpy = true;
  if (s[0] == 'a' && s[1] == 'a')
    use_aa = true;

  int nv = 0;
  double v[4];
  const char *last_e = s;
  for (; *s ; ++s)
  {
    if (*s == '*')
      break;
    char *e = NULL;
    v[nv] = strtod(s, &e);
    if (e == s)
      continue;
    s = e;
    last_e = e;
    nv++;
    if (nv == 4)
      break;
  }

  // reset s to right after last number read
  s = last_e;

  while (*s == ' ')
    s++;
  bool use_degrees = false;
  if (*s == 'd')
    use_degrees = true;

  // ignore the rest of the line up to a * if any.
  // * means multiply by another (following) rotation.
  while (*s && *s != '*')
    s++;

  *ss = s;

  if (nv == 1)
  {
    g_prec = int(v[0]);
    g_width = g_prec + 3;
    printf("Setting width=%d precision=%d", g_width, g_prec);
    return false;
  }
  else if (nv == 2)
  {
    g_width = int(v[0]);
    g_prec = int(v[1]);
    printf("Setting width=%d precision=%d", g_width, g_prec);
    return false;
  }
  else if (nv == 4 && use_aa)
  {
    double angle = v[3];
    if (use_degrees)
      angle *= PI / 180.0;
    *result = tf::Quaternion(tf::Vector3(v[0], v[1], v[2]), angle);
    if (g_debug)
    {
      printf("Using aa=%*.*f %*.*f %*.*f %*.*f (%*.*f deg)\n",
        g_width, g_prec, v[0],
        g_width, g_prec, v[1],
        g_width, g_prec, v[2],
        g_width, g_prec, angle,
        g_width, g_prec, angle * 180.0/PI);
    }
  }
  else if (nv == 4)
  {
    *result = tf::Quaternion(v[0], v[1], v[2], v[3]);
    if (g_debug)
    {
      printf("Using qx=%*.*f qy=%*.*f qz=%*.*f qw=%*.*f\n",
        g_width, g_prec, v[0],
        g_width, g_prec, v[1],
        g_width, g_prec, v[2],
        g_width, g_prec, v[3]);
    }
  }
  else if (nv == 3)
  {
    if (use_degrees)
    {
      v[0] *= PI / 180.0;
      v[1] *= PI / 180.0;
      v[2] *= PI / 180.0;
    }
    if (use_rpy)
    {
      result->setRPY(v[0], v[1], v[2]);
      if (g_debug)
      {
        printf("Using rz=%*.*f ry=%*.*f rx=%*.*f\n",
          g_width, g_prec, v[2],
          g_width, g_prec, v[1],
          g_width, g_prec, v[0]);
        printf("Using rz=%*.*f ry=%*.*f rx=%*.*f degrees\n",
          g_width, g_prec, v[2] * 180.0/PI,
          g_width, g_prec, v[1] * 180.0/PI,
          g_width, g_prec, v[0] * 180.0/PI);
      }
    }
    else
    {
      result->setRPY(v[2], v[1], v[0]);
      if (g_debug)
      {
        printf("Using rz=%*.*f ry=%*.*f rx=%*.*f\n",
          g_width, g_prec, v[0],
          g_width, g_prec, v[1],
          g_width, g_prec, v[2]);
        printf("Using rz=%*.*f ry=%*.*f rx=%*.*f degrees\n",
          g_width, g_prec, v[0] * 180.0/PI,
          g_width, g_prec, v[1] * 180.0/PI,
          g_width, g_prec, v[2] * 180.0/PI);
      }
    }
  }
  else
  {
    printf("Bad input line[%ld]: '%s'\n", long(s-line), line);
    return false;
  }

  return true;
}


int main(int argc, char *argv[])
{
  g_width = g_prec + 3;
  printf("INSTRUCTIONS:\n");
  printf("Setting width=%d precision=%d  (enter 1 or 2 numbers to set)\n",
    g_width, g_prec);
  printf("Quaternion: qx qy qz qw\n");
  printf("YPR:        rz ry rx (static transform publisher order)\n");
  printf("URDF:       rpy=rx ry rz   (NOTE: still applied in order rz, ry, rx)\n");
  printf("Axis Angle: aa=ax ay az angle  (follow angle by \"d\" for degrees)\n");
  printf("Use * to concatenate rotations.\n");
  for (;;)
  {
    printf("#######################\n");
    printf("Enter qx qy qz qw  or  rz ry rx : ");
    std::string sline;
    std::getline(std::cin, sline);
    double v[4];
    int nv = 0;
    sline += ";"; // ensure there is a terminator
    const char *line = sline.c_str();
    const char *s = line;

    tf::Quaternion q;
    if (readRot(&s, &q))
    {
      show(q);

      while (*s == '*')
      {
        s++;
        tf::Quaternion q2;
        if (readRot(&s, &q2))
        {
          printf("------- q2:\n");
          show(q2);
          printf("------- PRODUCT:\n");
          
          tf::Quaternion q3 = q * q2;
          show(q3);
          q = q3;
        }
        else
          break;
      }
    }
  }
}

