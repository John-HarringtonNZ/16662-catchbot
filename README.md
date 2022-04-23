# 16662-catchbot


Startup notes:


# Start FrankaPy:
`bash bash_scripts/start_control_psh -u student -i iam-grumpy -g 0`


Reset Arm: 
NOTE: Changes required to script are below:

```
    print('Starting robot')
    fa = FrankaArm(with_gripper=False)

    if args.use_pose:
        print('Reset with pose')
        fa.reset_pose()
    else:
        print('Reset with joints')
        fa.reset_joints()
    
    # if args.close_grippers:
    #     print('Closing Grippers')
    #     fa.close_gripper()
    # else:
    #     print('Opening Grippers')
    #     fa.open_gripper()
```

Run:
`python scripts/reset_arm.py`
