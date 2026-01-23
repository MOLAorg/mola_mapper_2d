/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this odometry package
 alone or in combination with the complete SLAM system.
*/

/**
 * @file   register.cpp
 * @brief  Register MOLA modules in the factory
 * @author Jose Luis Blanco Claraco
 * @date   Jan 23, 2026
 */

#include <mola_mapper_2d/Mapper2D.h>
#include <mrpt/core/initializer.h>
#include <mrpt/rtti/CObject.h>

MRPT_INITIALIZER(do_register_mola_mapper_2d)
{
  using mrpt::rtti::registerClass;

  // Register modules:
  MOLA_REGISTER_MODULE(mola::Mapper2D);
}
