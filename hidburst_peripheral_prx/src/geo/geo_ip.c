/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 * author: johnny nguyen
 */

#include "geo_ip.h"

static int8_t circle_coordinates[] =
    {
#include "circle.h"
};

void square_test_get(int8_t *ret_data)
{
  static uint8_t corner = 0;
  switch (corner)
  {
    case 0:
      ret_data[X_POS] = -4;
      ret_data[Y_POS] = -4;
      break;
    case 1:
      ret_data[X_POS] = -4;
      ret_data[Y_POS] = 4;
      break;
    case 2:
      ret_data[X_POS] = 4;
      ret_data[Y_POS] = 4;
      break;
    case 3:
      ret_data[X_POS] = 0;
      ret_data[Y_POS] = -4;
      break;
    default:
      break;
  }
}

void circle_test_get(int8_t *ret_data)
{
  static uint8_t rad = 0, cord_index = 0;

  // Uses the same data set for every radian of the circle, only altering the sign
  switch (rad)
  {
  case 0:
    ret_data[X_POS] = -circle_coordinates[CORD_SIZE - 1 - cord_index];
    ret_data[Y_POS] = circle_coordinates[cord_index];
    cord_index++;
    break;
  case 1:
    ret_data[X_POS] = -circle_coordinates[CORD_SIZE - 1 - cord_index];
    ret_data[Y_POS] = -circle_coordinates[cord_index];
    cord_index--;
    break;
  case 2:
    ret_data[X_POS] = circle_coordinates[CORD_SIZE - 1 - cord_index];
    ret_data[Y_POS] = -circle_coordinates[cord_index];
    cord_index++;
    break;
  case 3:
    ret_data[X_POS] = circle_coordinates[CORD_SIZE - 1 - cord_index];
    ret_data[Y_POS] = circle_coordinates[cord_index];
    cord_index--;
    break;
  }

  if (!(cord_index < CORD_SIZE))
  {
    rad = (rad + 1) % 4;
    if (rad % 2)
    {
      cord_index = (CORD_SIZE - 1);
    }
    else
    {
      cord_index = 0;
    }
  }
}
