/*
 * openRatSLAM
 *
 * utils - General purpose utility helper functions mainly for angles and readings settings
 *
 * Copyright (C) 2012
 * David Ball (david.ball@qut.edu.au) (1), Scott Heath (scott.heath@uqconnect.edu.au) (2)
 *
 * RatSLAM algorithm by:
 * Michael Milford (1) and Gordon Wyeth (1) ([michael.milford, gordon.wyeth]@qut.edu.au)
 *
 * 1. Queensland University of Technology, Australia
 * 2. The University of Queensland, Australia
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * The LocalViewMatch class is responsible for looking up images
 * and finding the best matching image and a score for how well the image
 * matches.
 */

#ifndef _VISUAL_TEMPLATE_MATCH_H_
#define _VISUAL_TEMPLATE_MATCH_H_

#include <cstring>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <iostream>
#include <vector>
#include <fstream>

#define _USE_MATH_DEFINES
#include "math.h"

#include <boost/property_tree/ini_parser.hpp>
using boost::property_tree::ptree;

#include <boost/serialization/access.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>

namespace ratslam
{

/** @brief 用于记录泛视模板，包括id, vt, rt **/
struct SuperTemplate
{
    unsigned int id;
    VisualTemplate vt;
    RangeTemplate rt;
};

/** @brief 用于记录深度模板的数据，包括id，data，以及mean，均值用于提高模板匹配的效率，如果发现均值差别较大，直接跳过 **/
struct RangeTemplate
{
    std::vector<double> data;
    double mean;
    double entropy;

    template<typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & data;
        ar & mean;
    }
};

/** @brief 用于记录视觉模板的数据，包括id，data，以及mean，均值用于提高模板匹配的效率，如果发现均值差别较大，直接跳过 **/
struct VisualTemplate
{
  //unsigned int id;
  std::vector<double> data;
  double mean;
  double entropy;

  template<typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
//      ar & id;
      ar & data;
      ar & mean;
    }

};

class LocalViewMatch
{
public:
  friend class LocalViewScene;

  LocalViewMatch(ptree settings);

  ~LocalViewMatch();

  /** @brief 用于响应新的传感器数据，目前包括灰度图通道和深度信息通道，后面可以加入激光雷达数据 **/
  void on_new_data(const unsigned char *img_grey, unsigned int img_grey_width, unsigned int img_grey_height,
                   const unsigned char *img_range,unsigned int img_range_width, unsigned int img_range_height);

  void on_image(const unsigned char *view_rgb, bool greyscale, unsigned int image_width, unsigned int image_height);

  int get_current_vt()
  {
    return current_vt;
  }

  double get_relative_rad()
  {
    return vt_relative_rad;
  }

  template<typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
      ar & VT_SHIFT_MATCH;
      ar & VT_STEP_MATCH;
      ar & VT_MATCH_THRESHOLD;
      ar & TEMPLATE_SIZE;
      ar & IMAGE_WIDTH;
      ar & IMAGE_HEIGHT;
      ar & IMAGE_VT_X_RANGE_MIN;
      ar & IMAGE_VT_X_RANGE_MAX;
      ar & IMAGE_VT_Y_RANGE_MIN;
      ar & IMAGE_VT_Y_RANGE_MAX;
      ar & TEMPLATE_X_SIZE;
      ar & TEMPLATE_Y_SIZE;
      ar & VT_PATCH_NORMALISATION;
      ar & VT_MIN_PATCH_NORMALISATION_STD;
      ar & VT_NORMALISATION;
      ar & VT_PANORAMIC;
 /** @todo 需要根据泛视模板修改**/
      ar & templates;
      ar & current_view;
      ar & current_vt_mean;
      ar & current_dt_mean;
      ar & image_size;
      ar & current_vt;
      ar & vt_error;
      ar & prev_vt;
      ar & vt_relative_rad;

    }

private:
  friend class boost::serialization::access;

  LocalViewMatch()
  {
    ;
  }
  void clip_view_x_y(int &x, int &y);

  // create and add a visual template to the collection
  int create_template();

  /** @brief 创建泛视模板 **/
  int create_super_template();

  /** @brief 设置当前泛视模板的id，并保存当前泛视模板的id至prev_st **/
  void set_current_st(int id)
  {
      if (current_st != id)
          prev_st = current_st;

      current_st = id;

  }

  void set_current_vt(int id)
  {
    if (current_vt != id)
      prev_vt = current_vt;

    current_vt = id;
  }

  /** calculate entropy of grey image */
  void calculate_img_grey_entropy();

  /** convert grey image to view template */
  void convert_view_to_view_template(bool grayscale);

  /** convert range image to range image template */
  void convert_range_to_range_template();

  /** calculate range image entropy */
  void calculate_img_range_entropy();

  // compare a visual template to all the stored templates, allowing for
  // slen pixel shifts in each direction
  // returns the matching template and the MSE
  void compare(double &vt_err, unsigned int &vt_match_id);

  /** compare a view and depth template pair against all the stored template pairs,
   * allowing for slen pixel shifts in each direction
   * return the matching template pair and the MSE
   */
  void compare_super_templates(double &vt_err, unsigned int &vt_match_id);

  /** 模板匹配过程中的两个重要参数，VT和ST可能会使用同一组参数 **/
  int VT_SHIFT_MATCH;
  int VT_STEP_MATCH;
  int ST_SHIFT_MATCH;
  int ST_STEP_MATCH;

  double ST_MATCH_THRESHOLD;
  double VT_MATCH_THRESHOLD;
  int TEMPLATE_SIZE;
  int IMAGE_WIDTH;
  int IMAGE_HEIGHT;
  int IMAGE_VT_X_RANGE_MIN;
  int IMAGE_VT_X_RANGE_MAX;
  int IMAGE_VT_Y_RANGE_MIN;
  int IMAGE_VT_Y_RANGE_MAX;
  int TEMPLATE_X_SIZE;
  int TEMPLATE_Y_SIZE;
  int VT_PATCH_NORMALISATION;
  double VT_MIN_PATCH_NORMALISATION_STD;
  double VT_NORMALISATION;
  int VT_PANORAMIC;

  std::vector<VisualTemplate> templates;
  std::vector<double> current_view;
  std::vector<double> current_depth;
  std::vector<double> current_relative_depth;

  /** entropy related variables */
  double current_view_entropy;
  double current_depth_entropy;

  int image_size;
  /** 当前泛视模板的id **/
  int current_st;
  int current_vt;
  /** 当前vt和st的均值 **/
  double current_vt_mean;
  double current_rt_mean;
  /** 两个vt, rt, st之间的差，注意：st_error不是vt_error和rt_error的相加 **/
  double vt_error;
  double rt_error;
  double st_error;
  /** 用于保存前一个泛视模板的id **/
  int prev_st;
  int prev_vt;
  double vt_relative_rad;

  const unsigned char *view_rgb;
  const unsigned char *depth;
  bool greyscale;

};

} // namespace ratslam

#endif // _VISUAL_TEMPLATE_MATCH_H_
