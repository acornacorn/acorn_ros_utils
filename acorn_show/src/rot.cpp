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
#include <ctype.h>

static const double PI = (boost::math::constants::pi<double>());

int g_prec = 10;
int g_width = 0;

bool g_debug = false;

// canonicalize quaternion
//   - normalize
//   - ensure w >= 0
static void canonicalize(tf::Quaternion& q)
{
  q.normalize();
  if (q.w() < 0.0)
    q *= -1.0;
}

// Canonicalize angle
// ensure between PI and -PI
static double canonAngle(double a)
{
  while(a > PI)
    a -= 2.0 * PI;
  while(a <= -PI)
    a += 2.0 * PI;
  return a;
}

static void MatrixToYPR(
  const tf::Matrix3x3& m,
  double *yaw,
  double *pitch,
  double *roll,
  int solution=0)
{
  double x,y,z = 0;
  if (std::abs(m[2][0]) < 1.0)
  {
    y = -asin(m[2][0]);
    if (solution)
      y = PI - y;
    double cy = cos(y);
    x = atan2(m[2][1]/cy, m[2][2]/cy);
    z = atan2(m[1][0]/cy, m[0][0]/cy);
  }

  else if (m[2][0] < 0.0)
  {
    y = PI * 0.5;
    x = atan2(m[0][1], m[0][2]);
    if (g_debug)
    {
      printf("~~~~~ case 2    WARNING tf::getEulerYPR BROKEN FOR THIS CASE\n");
      printf("x = atan2(m[0][1], m[0][2]) = atan2(%f , %f) = %f\n",
        m[0][1], m[0][2], x);
    }

  }
  else
  {
    y = -PI * 0.5;
    x = atan2(-m[0][1], -m[0][2]);
    if (g_debug)
    {
      printf("~~~~~ case 3    WARNING tf::getEulerYPR BROKEN FOR THIS CASE\n");
      printf("x = atan2(-m[0][1], -m[0][2]) = atan2(%f , %f) = %f\n",
        -m[0][1], -m[0][2], x);
    }
  }
    
  *roll = x;    // psi
  *pitch = y;   // theta
  *yaw = z;     // phi
}

#define QUAT_CHECK_EQUAL(q1,q2) \
        chackQuatEqual(q1,q2,#q1,#q2,__FILE__,__LINE__)
static bool chackQuatEqual(
      const tf::Quaternion& q1,
      const tf::Quaternion& q2,
      const std::string& name1,
      const std::string& name2,
      const char *file,
      int line)
{
  if (std::abs(q1.angleShortestPath(q2)) >
    10.0 * std::numeric_limits<double>::epsilon())
  {
    printf("ERROR: QUATERNION MISMATCH at %s:%d\n",
      file,line);
    printf("    %20s: qx=%*.*f qy=%*.*f qz=%*.*f qw=%*.*f\n",
      name1.c_str(),
      g_width, g_prec, q1.x(),
      g_width, g_prec, q1.y(),
      g_width, g_prec, q1.z(),
      g_width, g_prec, q1.w());
    printf("    %20s: qx=%*.*f qy=%*.*f qz=%*.*f qw=%*.*f\n",
      name2.c_str(),
      g_width, g_prec, q2.x(),
      g_width, g_prec, q2.y(),
      g_width, g_prec, q2.z(),
      g_width, g_prec, q2.w());
    return false;
  }
  return true;
}

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

  double angle = canonAngle(q.getAngle());
  tf::Vector3 axis = q.getAxis();
  {
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
      g_width, g_prec, canonAngle(angle),
      2+g_width, g_prec, canonAngle(angle) * 180.0 / PI);
  }

  {
    double angle = canonAngle(q.getAngleShortestPath());
    tf::Vector3 axis = q.getAxis();
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
      g_width, g_prec, canonAngle(angle),
      2+g_width, g_prec, canonAngle(angle) * 180.0 / PI);
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

  printf("YPR:  yaw_z=%*.*f pitch_y=%*.*f roll_x=%*.*f  (solution 1)\n",
    2+g_width, g_prec, canonAngle(yaw_z1),
    2+g_width, g_prec, canonAngle(pitch_y1),
    2+g_width, g_prec, canonAngle(roll_x1));
  printf("YPR:  yaw_z=%*.*f pitch_y=%*.*f roll_x=%*.*f  (solution 2)\n",
    2+g_width, g_prec, canonAngle(yaw_z2),
    2+g_width, g_prec, canonAngle(pitch_y2),
    2+g_width, g_prec, canonAngle(roll_x2));

  printf("YPR:  yaw_z=%*.*f pitch_y=%*.*f roll_x=%*.*f  (s1 degrees)\n",
    2+g_width, g_prec, canonAngle(yaw_z1) * 180.0/PI,
    2+g_width, g_prec, canonAngle(pitch_y1) * 180.0/PI,
    2+g_width, g_prec, canonAngle(roll_x1) * 180.0/PI);
  printf("YPR:  yaw_z=%*.*f pitch_y=%*.*f roll_x=%*.*f  (s2 degrees)\n",
    2+g_width, g_prec, canonAngle(yaw_z2) * 180.0/PI,
    2+g_width, g_prec, canonAngle(pitch_y2) * 180.0/PI,
    2+g_width, g_prec, canonAngle(roll_x2) * 180.0/PI);

  printf("URDF syntax:                rpy=\"%*.*f %*.*f %*.*f\"\n",
    g_width, g_prec, canonAngle(roll_x1),
    g_width, g_prec, canonAngle(pitch_y1),
    g_width, g_prec, canonAngle(yaw_z1));
  printf("static_transform_publisher: 0 0 0 %*.*f %*.*f %*.*f\n",
    g_width, g_prec, canonAngle(yaw_z1),
    g_width, g_prec, canonAngle(pitch_y1),
    g_width, g_prec, canonAngle(roll_x1));


  bool show_debug = g_debug;
  if (1)
  {
    double e_yaw_z1, e_pitch_y1, e_roll_x1;
    double e_yaw_z2, e_pitch_y2, e_roll_x2;
    m.getEulerYPR(e_yaw_z1, e_pitch_y1, e_roll_x1, 1);
    m.getEulerYPR(e_yaw_z2, e_pitch_y2, e_roll_x2, 2);

    tf::Quaternion mq;
    m.getRotation(mq);

    tf::Quaternion aaq(axis, angle);
    canonicalize(aaq);

    tf::Quaternion rpyq;
    rpyq.setRPY(roll_x1, pitch_y1, yaw_z1);
    canonicalize(rpyq);

    tf::Quaternion rpyq2;
    rpyq2.setRPY(roll_x2, pitch_y2, yaw_z2);
    canonicalize(rpyq2);

    bool err = false;

    if (!QUAT_CHECK_EQUAL(q,mq))
      show_debug = true;
    if (!QUAT_CHECK_EQUAL(q,aaq))
      show_debug = true;
    if (!QUAT_CHECK_EQUAL(q,rpyq))
      show_debug = true;
    if (!QUAT_CHECK_EQUAL(q,rpyq2))
      show_debug = true;
  }
  if (show_debug)
  {
    printf("----EXTRA DEBUG INFO FOLLOWS----\n");

    double e_yaw_z1, e_pitch_y1, e_roll_x1;
    m.getEulerYPR(e_yaw_z1, e_pitch_y1, e_roll_x1, 1);
    printf("EYPR: yaw_z=%*.*f pitch_y=%*.*f roll_x=%*.*f  (solution 1)\n",
      g_width, g_prec, canonAngle(e_yaw_z1),
      g_width, g_prec, canonAngle(e_pitch_y1),
      g_width, g_prec, canonAngle(e_roll_x1));
    double e_yaw_z2, e_pitch_y2, e_roll_x2;
    m.getEulerYPR(e_yaw_z2, e_pitch_y2, e_roll_x2, 2);
    printf("EYPR: yaw_z=%*.*f pitch_y=%*.*f roll_x=%*.*f  (solution 2)\n",
      g_width, g_prec, canonAngle(e_yaw_z2),
      g_width, g_prec, canonAngle(e_pitch_y2),
      g_width, g_prec, canonAngle(e_roll_x2));

    printf("EYPR: yaw_z=%*.*f pitch_y=%*.*f roll_x=%*.*f  (s1 degrees)\n",
      2+g_width, g_prec, canonAngle(e_yaw_z1) * 180.0/PI,
      2+g_width, g_prec, canonAngle(e_pitch_y1) * 180.0/PI,
      2+g_width, g_prec, canonAngle(e_roll_x1) * 180.0/PI);
    printf("EYPR: yaw_z=%*.*f pitch_y=%*.*f roll_x=%*.*f  (s2 degrees)\n",
      2+g_width, g_prec, canonAngle(e_yaw_z2) * 180.0/PI,
      2+g_width, g_prec, canonAngle(e_pitch_y2) * 180.0/PI,
      2+g_width, g_prec, canonAngle(e_roll_x2) * 180.0/PI);


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
  }
}

bool getMoreChars(std::string* sline,
                  const char **s1,
                  const char **s2,
                  const char **s3)
{
  size_t s1p = *s1 - sline->c_str();
  size_t s2p = *s2 - sline->c_str();
  size_t s3p = *s3 - sline->c_str();
  
  std::string sline2;
  std::getline(std::cin, sline2);
  if (sline2.empty() || (sline2.size() == 1 && sline2[0] == '\n'))
  {
    return false;
  }

  *sline = *sline + ";" + sline2 + ";";
  *s1 = sline->c_str() + s1p;
  *s2 = sline->c_str() + s2p;
  *s3 = sline->c_str() + s3p;
  return true;
}

bool readRot(const char **ss, tf::Quaternion *result, std::string* sline)
{
  const char *line = *ss;
  const char *s = line;

  while (*s == ' ')
    s++;

  bool use_rpy = false;
  bool use_aa = false;
  bool use_mtx = false;
  if (tolower(s[0]) == 'r' && tolower(s[1]) == 'p' && tolower(s[2]) == 'y')
    use_rpy = true;
  if (tolower(s[0]) == 'a' && tolower(s[1]) == 'a')
    use_aa = true;
  if (tolower(s[0]) == 'm' && tolower(s[1]) == '=')
    use_mtx = true;
  if (tolower(s[0]) == 'm' && tolower(s[1]) == 't' && tolower(s[2]) == 'x')
    use_mtx = true;
  if (tolower(s[0]) == 'd' && tolower(s[1]) == 'b' && tolower(s[2]) == 'g')
  {
    g_debug = !g_debug;
    printf("Debug mode is now %s\n",
      g_debug ? "ON" : "OFF");
  }

  int nv = 0;
  double v[9];
  const char *last_e = s;
  for (;; ++s)
  {
    if (use_mtx && !*s && (nv==3 || nv==6))
    {
      if (!getMoreChars(sline, &s, &line, &last_e))
      {
        printf("Bad input line[%ld]: '%s'\n", long(s-line), line);
        return false;
      }
    }
    if (!*s)
      break;
    if (*s == '*')
      break;
    char *e = NULL;
    v[nv] = strtod(s, &e);
    if (e == s)
      continue;
    s = e;
    last_e = e;
    nv++;
    if (nv == 4 && !use_mtx)
      break;
    if (nv == 9)
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
        g_width, g_prec, canonAngle(angle),
        2+g_width, g_prec, canonAngle(angle) * 180.0/PI);
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
          g_width, g_prec, canonAngle(v[2]),
          g_width, g_prec, canonAngle(v[1]),
          g_width, g_prec, canonAngle(v[0]));
        printf("Using rz=%*.*f ry=%*.*f rx=%*.*f degrees\n",
          2+g_width, g_prec, canonAngle(v[2]) * 180.0/PI,
          2+g_width, g_prec, canonAngle(v[1]) * 180.0/PI,
          2+g_width, g_prec, canonAngle(v[0]) * 180.0/PI);
      }
    }
    else
    {
      result->setRPY(v[2], v[1], v[0]);
      if (g_debug)
      {
        printf("Using rz=%*.*f ry=%*.*f rx=%*.*f\n",
          g_width, g_prec, canonAngle(v[0]),
          g_width, g_prec, canonAngle(v[1]),
          g_width, g_prec, canonAngle(v[2]));
        printf("Using rz=%*.*f ry=%*.*f rx=%*.*f degrees\n",
          2+g_width, g_prec, canonAngle(v[0]) * 180.0/PI,
          2+g_width, g_prec, canonAngle(v[1]) * 180.0/PI,
          2+g_width, g_prec, canonAngle(v[2]) * 180.0/PI);
      }
    }
  }
  else if (nv==9)
  {
    tf::Matrix3x3 m(
        v[0], v[1], v[2],
        v[3], v[4], v[5],
        v[6], v[7], v[8]);
    m.getRotation(*result);
    if (g_debug)
    {
      printf("Using mtx= %*.*f %*.*f %*.*f\n",
        g_width, g_prec, v[0],
        g_width, g_prec, v[1],
        g_width, g_prec, v[2]);
      printf("           %*.*f %*.*f %*.*f\n",
        g_width, g_prec, v[3],
        g_width, g_prec, v[4],
        g_width, g_prec, v[5]);
      printf("           %*.*f %*.*f %*.*f\n",
        g_width, g_prec, v[6],
        g_width, g_prec, v[7],
        g_width, g_prec, v[8]);
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

  printf("\n");
  printf("INSTRUCTIONS:\n");
  printf("Enter a rotation or product of rotations.  Rotation is one of:\n");
  printf(" <qx> <qy> <qz> <qw>    - quaternion\n");
  printf(" <rz> <ry> <rx>         - eulerZYX"
                                  " (static transform publisher order)\n");
  printf(" rpy=<rx> <ry> <rz>     - URDF order\n");
  printf(" aa=<x> <y> <z> <a>     - axis angle\n");
  printf(" mtx=<xx> <xy> <xz> <yx> <yy> <yz> <zx> <zy> <zz>\n");
  printf("                        - matrix (can optionally span 3 lines)\n");
  printf(" <precision>                    - set precision for output\n");

  printf("Extra (non numeric) chars before/between/after each number are"
        " ignored.\n");
  printf("End rotation with a d for degrees (otherwise radians assumed)\n");
  printf("Concat multiple rotations with * to see product\n");
  printf("NOTE: both YPR and URDF are **applied** in rz*ry*rx order\n");
  printf("To enable debug type: dbg\n");
  printf("Setting width=%d precision=%d  (enter 1 or 2 numbers to set)\n",
    g_width, g_prec);
  printf("\n");

  for (;;)
  {
    printf("#######################\n");
    printf("Enter qx qy qz qw  or  rz ry rx : ");
    std::string sline;
    std::getline(std::cin, sline);
    sline += ";"; // ensure there is a terminator
    const char *s = sline.c_str();

    tf::Quaternion q;
    if (readRot(&s, &q, &sline))
    {
      show(q);

      while (*s == '*')
      {
        s++;
        tf::Quaternion q2;
        if (readRot(&s, &q2, &sline))
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

