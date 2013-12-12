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

static void show(tf::Quaternion q)
{
  printf("Quat: qx=%*.*f qy=%*.*f qz=%*.*f qw=%*.*f\n",
    g_width, g_prec, q.x(),
    g_width, g_prec, q.y(),
    g_width, g_prec, q.z(),
    g_width, g_prec, q.w());
  q.normalize();
  printf("Quat: qx=%*.*f qy=%*.*f qz=%*.*f qw=%*.*f (normalized)\n",
    g_width, g_prec, q.x(),
    g_width, g_prec, q.y(),
    g_width, g_prec, q.z(),
    g_width, g_prec, q.w());

  {
    double angle = q.getAngle();
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
  m.getRPY(roll_x1, pitch_y1, yaw_z1, 1);
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
  double yaw_z2, pitch_y2, roll_x2;
  m.getRPY(roll_x2, pitch_y2, yaw_z2, 2);
  printf("YPR:  yaw_z=%*.*f pitch_y=%*.*f roll_x=%*.*f  (solution 2)\n",
    g_width, g_prec, yaw_z2,
    g_width, g_prec, pitch_y2,
    g_width, g_prec, roll_x2);

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
  for (; *s ; ++s)
  {
    if (*s == '*')
      break;
    char *e = NULL;
    v[nv] = strtod(s, &e);
    if (e == s)
      continue;
    s = e;
    nv++;
    if (nv == 4)
      break;
  }

  while (*s == ' ')
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
    if (*s == 'd')
      angle *= PI / 180.0;
    *result = tf::Quaternion(tf::Vector3(v[0], v[1], v[2]), angle);
  }
  else if (nv == 4)
  {
    *result = tf::Quaternion(v[0], v[1], v[2], v[3]);
  }
  else if (nv == 3)
  {
    if (use_rpy)
      result->setRPY(v[0], v[1], v[2]);
    else
      result->setRPY(v[2], v[1], v[0]);
  }
  else
  {
    printf("Bad input line[%ld]: '%s'\n", long(s-line), line);
    return false;
  }

  while (*s && *s != '*')
    s++;

  *ss = s;
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

