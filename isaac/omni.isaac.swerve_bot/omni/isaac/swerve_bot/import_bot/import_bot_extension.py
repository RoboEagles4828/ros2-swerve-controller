# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import os
from omni.isaac.swerve_bot.base_sample import BaseSampleExtension
from omni.isaac.swerve_bot.import_bot.import_bot import ImportBot


class ImportBotExtension(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)
        super().start_extension(
            menu_name="Swerve Bot",
            submenu_name="",
            name="Import URDF",
            title="Load the URDF for Swerve Bot",
            doc_link="",
            overview="This loads the Swerve bot into Isaac Sim.",
            file_path=os.path.abspath(__file__),
            sample=ImportBot(),
        )
        return
