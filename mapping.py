#!/usr/bin/env python3
"""Vendor laser SLAM workflow; commands preview locally unless --execute is given."""
import argparse
import json
import sys
import time


def describe_map(item):
    return dict(id=int(item.id),name=str(item.name),is_current=bool(item.is_curr_map))


def record_map(slam, *, finish_prompt=input):
    """Keep SLAM alive while an operator drives using the separate chassis tool."""
    slam.start_mapping()
    saved = False
    try:
        finish_prompt('建图已开始。用另一个终端遥控底盘覆盖环境，回到此处按 Enter 停止并保存：')
        slam.stop_mapping()
        saved = True
    finally:
        if not saved:
            slam.cancel_mapping()


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=['maps','record','switch','localize'], nargs='?', default='maps')
    p.add_argument('--map-id', type=int)
    p.add_argument('--execute', action='store_true')
    args = p.parse_args(argv)
    if args.action == 'switch' and (args.map_id is None or not 0 <= args.map_id <= 255):
        p.error('switch requires --map-id in 0..255')
    if args.action == 'localize':
        p.error('use the Pad for relocalization; SDK global_local has a documented initial-mapping-pose restriction')
    if not args.execute:
        print(json.dumps(dict(mode='preview',sdk_imported=False,action=args.action,map_id=args.map_id,
                              moves_robot=False),indent=2))
        return 0
    if sys.version_info[:2] != (3,10): p.error('use robot-native Python 3.10 and SDK environment')
    import agibot_gdk as sdk
    initialized = False
    try:
        if sdk.gdk_init() != sdk.GDKRes.kSuccess: raise RuntimeError('gdk_init failed')
        initialized = True
        maps = sdk.Map()
        if args.action == 'record':
            slam = sdk.Slam(); time.sleep(2)
            record_map(slam)
            print(json.dumps(dict(status='mapping_stop_and_save_requested',physical_map_quality_verified=False)))
        elif args.action == 'switch':
            time.sleep(2); maps.switch_map(args.map_id)
            print(json.dumps(dict(status='map_switch_requested',map_id=args.map_id,
                                  relocalization_required=True)))
        else:
            time.sleep(2)
            print(json.dumps(dict(current=describe_map(maps.get_curr_map()),
                                  maps=[describe_map(m) for m in maps.get_all_map()]),indent=2))
        return 0
    except (KeyboardInterrupt, EOFError):
        print(json.dumps(dict(status='cancelled')))
        return 1
    except Exception as exc:
        print(json.dumps(dict(status='failed',error=str(exc))))
        return 1
    finally:
        if initialized:
            try: sdk.release_dds()
            finally: sdk.gdk_release()


if __name__ == '__main__':
    raise SystemExit(main())
