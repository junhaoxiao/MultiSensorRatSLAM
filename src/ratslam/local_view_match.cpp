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

#include "local_view_match.h"
#include "../utils/utils.h"

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <iostream>
#include <iomanip>
using namespace std;
#include <boost/foreach.hpp>
#include <algorithm>

#include <stdio.h>

namespace ratslam
{

  LocalViewMatch::LocalViewMatch(ptree settings)
  {
    get_setting_from_ptree(VT_MIN_PATCH_NORMALISATION_STD, settings, "vt_min_patch_normalisation_std", (double)0);
    get_setting_from_ptree(VT_PATCH_NORMALISATION, settings, "vt_patch_normalise", 0);
    get_setting_from_ptree(VT_NORMALISATION, settings, "vt_normalisation", (double) 0);
    get_setting_from_ptree(VT_SHIFT_MATCH, settings, "vt_shift_match", 25);
    get_setting_from_ptree(VT_STEP_MATCH, settings, "vt_step_match", 5);
    get_setting_from_ptree(VT_PANORAMIC, settings, "vt_panoramic", 0);

    get_setting_from_ptree(VT_MATCH_THRESHOLD, settings, "vt_match_threshold", 0.03);
    get_setting_from_ptree(TEMPLATE_X_SIZE, settings, "template_x_size", 1);
    get_setting_from_ptree(TEMPLATE_Y_SIZE, settings, "template_y_size", 1);
    get_setting_from_ptree(IMAGE_VT_X_RANGE_MIN, settings, "image_crop_x_min", 0);
    get_setting_from_ptree(IMAGE_VT_X_RANGE_MAX, settings, "image_crop_x_max", -1);
    get_setting_from_ptree(IMAGE_VT_Y_RANGE_MIN, settings, "image_crop_y_min", 0);
    get_setting_from_ptree(IMAGE_VT_Y_RANGE_MAX, settings, "image_crop_y_max", -1);

    TEMPLATE_SIZE = TEMPLATE_X_SIZE * TEMPLATE_Y_SIZE;

    templates.reserve(10000);

    current_view.resize(TEMPLATE_SIZE);

    current_vt = 0;
    prev_vt = 0;
  }


  LocalViewMatch::~LocalViewMatch()
  {

  }

  void LocalViewMatch::on_image(const unsigned char *view_rgb,
                                bool greyscale,
                                unsigned int image_width,
                                unsigned int image_height)
  {
    if (view_rgb == NULL)
      return;

    IMAGE_WIDTH = image_width;
    IMAGE_HEIGHT = image_height;

    if (IMAGE_VT_X_RANGE_MAX == -1)
      IMAGE_VT_X_RANGE_MAX = IMAGE_WIDTH;
    if (IMAGE_VT_Y_RANGE_MAX == -1)
      IMAGE_VT_Y_RANGE_MAX = IMAGE_HEIGHT;

    this->view_rgb = view_rgb;
    this->greyscale = greyscale;

    convert_view_to_view_template(greyscale);
    prev_vt = get_current_vt();
    unsigned int vt_match_id;
    compare(vt_error, vt_match_id);
    if (vt_error <= VT_MATCH_THRESHOLD)
    {
      set_current_vt((int)vt_match_id);
      cout << "VTM[" << setw(4) << get_current_vt() << "] " << endl;
      cout.flush();
    }
    else
    {
      vt_relative_rad = 0;
      set_current_vt(create_template());
      cout << "VTN[" << setw(4) << get_current_vt() << "] " << endl;
      cout.flush();
    }

  }


  void LocalViewMatch::on_depth(const unsigned char *depth,
                                unsigned int depth_width,
                                unsigned int depth_height)
  {
    if (depth == NULL)
      return;

    IMAGE_WIDTH = depth_width;
    IMAGE_HEIGHT = depth_height;

    if (IMAGE_VT_X_RANGE_MAX == -1)
      IMAGE_VT_X_RANGE_MAX = IMAGE_WIDTH;
    if (IMAGE_VT_Y_RANGE_MAX == -1)
      IMAGE_VT_Y_RANGE_MAX = IMAGE_HEIGHT;

    this->depth = view_rgb;

    convert_depth_to_depth_template();

    prev_vt = get_current_vt();
    unsigned int vt_match_id;
    compare(vt_error, vt_match_id);
    if (vt_error <= VT_MATCH_THRESHOLD)
    {
      set_current_vt((int)vt_match_id);
      cout << "VTM[" << setw(4) << get_current_vt() << "] " << endl;
      cout.flush();
    }
    else
    {
      vt_relative_rad = 0;
      set_current_vt(create_template());
      cout << "VTN[" << setw(4) << get_current_vt() << "] " << endl;
      cout.flush();
    }

  }


  void LocalViewMatch::convert_depth_to_depth_template()
  {
    int data_next = 0;
    int sub_range_x = IMAGE_VT_X_RANGE_MAX - IMAGE_VT_X_RANGE_MIN;
    int sub_range_y = IMAGE_VT_Y_RANGE_MAX - IMAGE_VT_Y_RANGE_MIN;
    int x_block_size = sub_range_x / TEMPLATE_X_SIZE;
    int y_block_size = sub_range_y / TEMPLATE_Y_SIZE;
    int pos;
    int valid_depth_num;

    for (unsigned int i; i < current_depth.size(); i++)
      current_depth[i] = 0;

    for (int y_block = IMAGE_VT_Y_RANGE_MIN, y_block_count = 0; y_block_count < TEMPLATE_Y_SIZE; y_block += y_block_size, y_block_count++)
    {
      for (int x_block = IMAGE_VT_X_RANGE_MIN, x_block_count = 0; x_block_count < TEMPLATE_X_SIZE; x_block += x_block_size, x_block_count++)
      {
        valid_depth_num = 0;
        for (int x = x_block; x < (x_block + x_block_size); x++)
        {
          for (int y = y_block; y < (y_block + y_block_size); y++)
          {
            pos = (x + y * IMAGE_WIDTH);
            if (depth[pos] == 0.0)
              continue;
            current_depth[data_next] += (double)(depth[pos]);
            valid_depth_num++;
          }
        }
        current_depth[data_next] /= valid_depth_num;
        data_next++;
      }
    }

    /** depth image relative encoding */
    current_relative_depth.resize(current_depth.size());
    for (unsigned int i = 0; i < current_relative_depth.size(); i++)
    {
      current_relative_depth[i] = 8;
    }

    int encoding;
    for (int x = 1; x < TEMPLATE_X_SIZE - 1; x++)
    {
      for (int y = 1; y < TEMPLATE_Y_SIZE - 1; y++)
      {
        encoding = 0;
        current_depth[x+y*TEMPLATE_X_SIZE] > current_depth[x - 1 + (y - 1) * TEMPLATE_X_SIZE]? encoding++:encoding--;
        current_depth[x+y*TEMPLATE_X_SIZE] > current_depth[x + (y -1) * TEMPLATE_X_SIZE]? encoding++:encoding--;
        current_depth[x+y*TEMPLATE_X_SIZE] > current_depth[x + 1 + (y - 1) * TEMPLATE_X_SIZE]? encoding++:encoding--;
        current_depth[x+y*TEMPLATE_X_SIZE] > current_depth[x + 1 + y * TEMPLATE_X_SIZE]? encoding++:encoding--;
        current_depth[x+y*TEMPLATE_X_SIZE] > current_depth[x + 1 + (y + 1) * TEMPLATE_X_SIZE]? encoding++:encoding--;
        current_depth[x+y*TEMPLATE_X_SIZE] > current_depth[x + (y + 1) * TEMPLATE_X_SIZE]? encoding++:encoding--;
        current_depth[x+y*TEMPLATE_X_SIZE] > current_depth[x -1 + (y + 1) * TEMPLATE_X_SIZE]? encoding++:encoding--;
        current_depth[x+y*TEMPLATE_X_SIZE] > current_depth[x -1 + y * TEMPLATE_X_SIZE]? encoding++:encoding--;
        current_relative_depth[x+y*TEMPLATE_X_SIZE] = static_cast<double>(encoding);
      }
    }

    double sum = 0;
    // find the mean of the data
    for (int i = 0; i < current_relative_depth.size(); i++)
      sum += current_relative_depth[i];

    /** 应该根据current_depth计算深度图像的Entropy */
    current_dt_mean = sum/current_relative_depth.size();
  }

  /** set x and y to valid index */
  void LocalViewMatch::clip_view_x_y(int &x, int &y)
  {
    if (x < 0)
      x = 0;
    else if (x > TEMPLATE_X_SIZE - 1)
      x = TEMPLATE_X_SIZE - 1;

    if (y < 0)
      y = 0;
    else if (y > TEMPLATE_Y_SIZE - 1)
      y = TEMPLATE_Y_SIZE - 1;

  }

  void LocalViewMatch::convert_view_to_view_template(bool grayscale)
  {
    int data_next = 0;
    int sub_range_x = IMAGE_VT_X_RANGE_MAX - IMAGE_VT_X_RANGE_MIN;
    int sub_range_y = IMAGE_VT_Y_RANGE_MAX - IMAGE_VT_Y_RANGE_MIN;
    int x_block_size = sub_range_x / TEMPLATE_X_SIZE;
    int y_block_size = sub_range_y / TEMPLATE_Y_SIZE;
    int pos;

    for (unsigned int i; i < current_view.size(); i++)
      current_view[i] = 0;

    if (grayscale)
    {
      for (int y_block = IMAGE_VT_Y_RANGE_MIN, y_block_count = 0; y_block_count < TEMPLATE_Y_SIZE; y_block += y_block_size, y_block_count++)
      {
        for (int x_block = IMAGE_VT_X_RANGE_MIN, x_block_count = 0; x_block_count < TEMPLATE_X_SIZE; x_block += x_block_size, x_block_count++)
        {
          for (int x = x_block; x < (x_block + x_block_size); x++)
          {
            for (int y = y_block; y < (y_block + y_block_size); y++)
            {
              pos = (x + y * IMAGE_WIDTH);
              current_view[data_next] += (double)(view_rgb[pos]);
            }
          }
          current_view[data_next] /= (255.0);
          current_view[data_next] /= (x_block_size * y_block_size);
          data_next++;
        }
      }
    }
    else
    {
      for (int y_block = IMAGE_VT_Y_RANGE_MIN, y_block_count = 0; y_block_count < TEMPLATE_Y_SIZE; y_block +=
           y_block_size, y_block_count++)
      {
        for (int x_block = IMAGE_VT_X_RANGE_MIN, x_block_count = 0; x_block_count < TEMPLATE_X_SIZE; x_block +=
             x_block_size, x_block_count++)
        {
          for (int x = x_block; x < (x_block + x_block_size); x++)
          {
            for (int y = y_block; y < (y_block + y_block_size); y++)
            {
              pos = (x + y * IMAGE_WIDTH) * 3;
              current_view[data_next] += ((double)(view_rgb[pos]) + (double)(view_rgb[pos + 1])
                  + (double)(view_rgb[pos + 2]));
            }
          }
          current_view[data_next] /= (255.0 * 3.0);
          current_view[data_next] /= (x_block_size * y_block_size);

          data_next++;
        }
      }
    }

    if (VT_NORMALISATION > 0)
    {
      double avg_value = 0;

      for (unsigned int i = 0; i < current_view.size(); i++)
      {
        avg_value += current_view[i];
      }

      avg_value /= current_view.size();

      /** ALL templates are normalized by scaling their mean to vt_normalisation, this addresses global changes
          in illumination. */
      for (unsigned int i = 0; i < current_view.size(); i++)
      {
        /** the following equals to: current_view[i] = std::min(current_view[i] * VT_NORMALISATION, 1.0),
         *  since current_view[i] * VT_NORMALISATION is always greater than 0.0. */
        current_view[i] = std::max(0.0, std::min(current_view[i] * VT_NORMALISATION / avg_value, 1.0));
      }
    }

    /** now do patch normalisation */
    /** +- patch size on the pixel, ie 4 will give a 9x9 */
    if (VT_PATCH_NORMALISATION > 0)
    {
      int patch_size = VT_PATCH_NORMALISATION;
      int patch_total = (patch_size * 2 + 1) * (patch_size * 2 + 1);
      double patch_sum;
      double patch_mean;
      double patch_std;
      int patch_x_clip;
      int patch_y_clip;

      // first make a copy of the view
      std::vector<double> current_view_copy;
      current_view_copy.resize(current_view.size());
      for (unsigned int i = 0; i < current_view.size(); i++)
        current_view_copy[i] = current_view[i];

      // this code could be significantly optimimised ....
      for (int x = 0; x < TEMPLATE_X_SIZE; x++)
      {
        for (int y = 0; y < TEMPLATE_Y_SIZE; y++)
        {
          patch_sum = 0;
          for (int patch_x = x - patch_size; patch_x < x + patch_size + 1; patch_x++)
          {
            for (int patch_y = y - patch_size; patch_y < y + patch_size + 1; patch_y++)
            {
              patch_x_clip = patch_x;
              patch_y_clip = patch_y;
              clip_view_x_y(patch_x_clip, patch_y_clip);

              patch_sum += current_view_copy[patch_x_clip + patch_y_clip * TEMPLATE_X_SIZE];
            }
          }
          patch_mean = patch_sum / patch_total;

          patch_sum = 0;
          for (int patch_x = x - patch_size; patch_x < x + patch_size + 1; patch_x++)
          {
            for (int patch_y = y - patch_size; patch_y < y + patch_size + 1; patch_y++)
            {
              patch_x_clip = patch_x;
              patch_y_clip = patch_y;
              clip_view_x_y(patch_x_clip, patch_y_clip);

              patch_sum += ((current_view_copy[patch_x_clip + patch_y_clip * TEMPLATE_X_SIZE] - patch_mean)
                  * (current_view_copy[patch_x_clip + patch_y_clip * TEMPLATE_X_SIZE] - patch_mean));
            }
          }

          patch_std = sqrt(patch_sum / patch_total);

          /** VT_MIN_PATCH_NORMALISATION_STD给的默认值是0，因此，从没进入过if条件，每次都执行else中的几个参数. */
          if (patch_std < VT_MIN_PATCH_NORMALISATION_STD)
          {
            current_view[x + y * TEMPLATE_X_SIZE] = 0.5;
          }
          else
          {
            /** 这里的几个参数是如何得到的？感觉没什么道理的. */
            current_view[x + y * TEMPLATE_X_SIZE] = max((double) 0, min(1.0, (((current_view_copy[x + y * TEMPLATE_X_SIZE] - patch_mean) / patch_std) + 3.0)/6.0 ));
          }
        }
      }
    }

    double sum = 0;

    // find the mean of the data
    for (int i = 0; i < current_view.size(); i++)
      sum += current_view[i];

    /** 应该根据current_view计算图像的Entropy */
    current_vt_mean = sum/current_view.size();

  }

  void LocalViewMatch::calculate_view_entropy()
  {
    double entropy;
    int hist[256] = {0};
    std::vector<int> current_view_copy;
    current_view_copy.resize(current_view.size());
    std::vector<int>::iterator iit;
    std::vector<double>::iterator dit;
    for (iit = current_view_copy.begin(), dit = current_view.begin(); iit < current_view_copy.end(); iit++, dit++)
    {
      *iit = static_cast<int>(*dit * 255.0);
    }

    for (iit = current_view_copy.begin(); iit < current_view_copy.end(); iit++)
    {
      hist[*iit] ++;
    }

    for (size_t i = 0; i < 256; i++)
    {
      entropy -= static_cast<double>(hist[i]/current_view_copy.size())*log(static_cast<double>(hist[i]/current_view_copy.size()));
    }

    current_view_entropy = entropy;
  }

  // create and add a visual template to the collection
  int LocalViewMatch::create_template()
  {
    templates.resize(templates.size() + 1);
    VisualTemplate * new_template = &(*(templates.end() - 1));

    new_template->id = templates.size() - 1;
    double * data_ptr = &current_view[0];
    new_template->data.reserve(TEMPLATE_SIZE);
    for (int i = 0; i < TEMPLATE_SIZE; i++)
      new_template->data.push_back(*(data_ptr++));

    new_template->mean = current_vt_mean;

    return templates.size() - 1;
  }

  void LocalViewMatch::compare_super_templates(double &vt_err, unsigned int &vt_match_id)
  {
    if (templates.size() == 0)
    {
      vt_err = DBL_MAX;
      vt_error = vt_err;
      return;
    }

    double *data = &current_view[0];
    test the origin master.
  }

  // compare a visual template to all the stored templates, allowing for
  // slen pixel shifts in each direction
  // returns the matching template and the MSE
  void LocalViewMatch::compare(double &vt_err, unsigned int &vt_match_id)
  {
    if (templates.size() == 0)
    {
      vt_err = DBL_MAX;
      vt_error = vt_err;
      return;
    }

    double *data = &current_view[0];
    double mindiff, cdiff;
    mindiff = DBL_MAX; //DBL_MAX: maximal of data type double

    vt_err = DBL_MAX;
    int min_template = 0;

    double *template_ptr;
    double *column_ptr;
    double *template_row_ptr;
    double *column_row_ptr;
    double *template_start_ptr;
    double *column_start_ptr;
    int row_size;
    int sub_row_size;
    double *column_end_ptr;
    VisualTemplate vt;
    int min_offset;

    int offset;
    double epsilon = 0.005;

    if (VT_PANORAMIC)
    {
      BOOST_FOREACH(vt, templates)
      {
        if (abs(current_vt_mean - vt.mean) > VT_MATCH_THRESHOLD + epsilon)
          continue;

        // for each vt try matching the view at different offsets
        // try to fast break based on error already great than previous errors
        // handles 2d images shifting only in the x direction
        // note I haven't tested on a 1d yet.
        for (offset = 0; offset < TEMPLATE_X_SIZE; offset += VT_STEP_MATCH)
        {
          cdiff = 0;
          template_start_ptr = &vt.data[0] + offset;
          column_start_ptr = &data[0];
          row_size = TEMPLATE_X_SIZE;
          column_end_ptr = &data[0] + TEMPLATE_SIZE - offset;
          sub_row_size = TEMPLATE_X_SIZE - offset;

          // do from offset to end
          for (column_row_ptr = column_start_ptr, template_row_ptr = template_start_ptr; column_row_ptr < column_end_ptr; column_row_ptr+=row_size, template_row_ptr+=row_size)
          {
            for (column_ptr = column_row_ptr, template_ptr = template_row_ptr; column_ptr < column_row_ptr + sub_row_size; column_ptr++, template_ptr++)
            {
              cdiff += abs(*column_ptr - *template_ptr);
            }

            // fast breaks
            if (cdiff > mindiff)
              break;
          }

          // do from start to offset
          template_start_ptr = &vt.data[0];
          column_start_ptr = &data[0] + TEMPLATE_X_SIZE - offset;
          row_size = TEMPLATE_X_SIZE;
          column_end_ptr = &data[0] + TEMPLATE_SIZE;
          sub_row_size = offset;
          for (column_row_ptr = column_start_ptr, template_row_ptr = template_start_ptr; column_row_ptr < column_end_ptr; column_row_ptr+=row_size, template_row_ptr+=row_size)
          {
            for (column_ptr = column_row_ptr, template_ptr = template_row_ptr; column_ptr < column_row_ptr + sub_row_size; column_ptr++, template_ptr++)
            {
              cdiff += abs(*column_ptr - *template_ptr);
            }

            // fast breaks
            if (cdiff > mindiff)
              break;
          }


          if (cdiff < mindiff)
          {
            mindiff = cdiff;
            min_template = vt.id;
            min_offset = offset;
          }
        }

      }

      vt_relative_rad = (double) min_offset/TEMPLATE_X_SIZE * 2.0 * M_PI;
      if (vt_relative_rad > M_PI)
        vt_relative_rad = vt_relative_rad - 2.0 * M_PI;
      vt_err = mindiff / (double) TEMPLATE_SIZE;
      vt_match_id = min_template;

      vt_error = vt_err;

    }
    else /** not VT_PANORAMIC */
    {
      BOOST_FOREACH(vt, templates)
      {
        /** fast break */
        if (abs(current_vt_mean - vt.mean) > VT_MATCH_THRESHOLD + epsilon)
          continue;

        // for each vt try matching the view at different offsets
        // try to fast break based on error already great than previous errors
        // handles 2d images shifting only in the x direction
        // note I haven't tested on a 1d yet.
        for (offset = 0; offset < VT_SHIFT_MATCH*2+1; offset += VT_STEP_MATCH)
        {
          cdiff = 0;
          template_start_ptr = &vt.data[0] + offset;
          column_start_ptr = &data[0] + VT_SHIFT_MATCH;
          row_size = TEMPLATE_X_SIZE;
          column_end_ptr = &data[0] + TEMPLATE_SIZE - VT_SHIFT_MATCH;
          sub_row_size = TEMPLATE_X_SIZE - 2*VT_SHIFT_MATCH;

          /** 看起来很乱，主要是指针操作 */
          for (column_row_ptr = column_start_ptr, template_row_ptr = template_start_ptr; column_row_ptr < column_end_ptr; column_row_ptr+=row_size, template_row_ptr+=row_size)
          {
            for (column_ptr = column_row_ptr, template_ptr = template_row_ptr; column_ptr < column_row_ptr + sub_row_size; column_ptr++, template_ptr++)
            {
              cdiff += abs(*column_ptr - *template_ptr);
            }

            // fast breaks
            if (cdiff > mindiff)
              break;
          }

          if (cdiff < mindiff)
          {
            mindiff = cdiff;
            min_template = vt.id;
            min_offset = 0;
          }
        }

      }

      vt_relative_rad = 0;
      vt_err = mindiff / (double)(TEMPLATE_SIZE - 2 * VT_SHIFT_MATCH * TEMPLATE_Y_SIZE);
      vt_match_id = min_template;

      vt_error = vt_err;

    }
  }

}
