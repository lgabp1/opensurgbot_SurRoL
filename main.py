import logging
import time
from typing import Dict

from deehli_surrol_wrapper import PipelineGeneric, PipelineDeehli, DeehliSurRoLWrapper

if __name__ == '__main__':
    from surrol.tasks.needle_pick import NeedlePick
    from surrol.tasks.needle_regrasp_bimanual import NeedleRegrasp
    surrol_task = NeedlePick(render_mode='human')
    logger = logging.Logger("", logging.DEBUG)
    logger.addHandler(logging.StreamHandler()) 

    pipelines: Dict[str, PipelineGeneric] = {} # Pipelines to insert
    pipelines['psm1'] = PipelineDeehli(serial_port="COM3", logger=logger)  # Override self.psm1's move and move_jaw methods
    
    if False:
        cangl = 0.0
        from deehli.lib.deehli import InverseKinematicsDescription
        while True:
            inp = input("New jaw angle: \n")
            if inp.startswith("c"):
                cangl = float(inp.strip("c"))
            else:
                angl = float(inp)

            angl_j1 = cangl - angl / 2
            angl_j2 = cangl + angl / 2
            
            pipelines["psm1"].move_action(InverseKinematicsDescription(0, 0, angl_j1, angl_j2, 0))

    deehli_surrol_wrapper = DeehliSurRoLWrapper(surrol_task, pipelines)

    surrol_task.test()
    print("Test finished !")
    input("Press Enter to continue...")
    surrol_task.close()
    time.sleep(2)

    # ISSUES TO SOLVE:
    # 2. Issue with move_jaw (closing seems not too work well)