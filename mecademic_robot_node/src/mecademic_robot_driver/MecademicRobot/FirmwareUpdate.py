#!/usr/bin/env python3
import sys
import argparse
import requests
import time
import json
from MecademicRobot import RobotController as RC

def get_args():
    """Command line interface to get arguments

    Command line interface to collect the robot IP address and the path
    of the firmware file.

    Returns
    -------
    string
        Path the the robot firmware file for the robot update.
    string
        HTTP address of the robot to update.

    """
    parser = argparse.ArgumentParser(
            description=f'Run the firmware Update of the robot.',
            epilog=f'exemple: python FirmwareUpdate.py --robot_fw_path ../../fw/v8.3.2.update --robot_ip_address m500-test-1.')
    parser.add_argument(
            '--robot_fw_path',
            metavar='robot_fw_path',
            type=str,
            nargs='+',
            default='.',
            help='The path of the firmware to update the robot.')
    parser.add_argument(
            '--robot_ip_address',
            metavar='robot_ip_address',
            type=str,
            nargs='+',
            default=['192.168.0.100'],
            help='The IP of the robot that will be update.')
    args = parser.parse_args(sys.argv[1:])
    return [args.robot_fw_path[0], args.robot_ip_address[0]]

def update_robot(file_path, ip_address):
    """Send the update specified by file_path to the robot.

    Parameters
    ----------
    Param file_path : string
        Path to the firware file
    Param ip_address: string
        IP Address of the robot to update

    """
    REQUEST_GET_TIMEOUT = 10
    ip_address_long = f'http://{ip_address}/'
    robot_inst = RC.RobotController(ip_address)
    if robot_inst.connect() :
        robot_inst.DeactivateRobot()
        robot_inst.disconnect()
        print(f'Opening firmware file...')
        try:
            update_file = open(file_path, 'rb')
        except OSError:
            print(f'Could not open/read file: {file_path}.')
            sys.exit()
        with update_file:
            update_data = update_file.read()
            update_file_size_str = str(len(update_data))
        print(f'Done, update size is: {update_file_size_str}B.')

        print(f'Uploading file...')
        headers = {'Connection': 'keep-alive',
                   'Content-type': 'application/x-gzip',
                   'Content-Length': update_file_size_str}
        r = requests.post(ip_address_long, data=update_data, headers=headers)
        try:
            r.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print(f'HTTP request error when posting Update request.')
            print(f'Error: {errh}')
            print(f'Firmware upload request failed.')
            sys.exit()
        except requests.exceptions.ConnectionError as errc:
            print(f'HTTP request connection error when posting Update request.')
            print(f'Error: {errc}')
            print(f'Firmware upload request failed.')
            sys.exit()
        except requests.exceptions.Timeout as errt:
            print(f'HTTP request timeout when posting Update request.')
            print(f'Error: {errt}')
            print(f'Firmware upload request failed.')
            sys.exit()
        except requests.exceptions.RequestException as err:
            print(f'HTTP request return a connection Error when posting Update request.')
            print(f'Error: {err}')
            print(f'Firmware upload request failed.')
            sys.exit()

        print('Upgrading the robot...')
        update_done = False
        progress = ''
        last_progress = ''
        while not update_done:
            r = requests.get(ip_address_long, 'update', timeout=REQUEST_GET_TIMEOUT)
            try:
                r.raise_for_status()
            except requests.exceptions.HTTPError as errh:
                print(f'HTTP request error when posting Update request.')
                print(f'Error: {errh}')
                print(f'Firmware upgrading failed.')
                sys.exit()
            except requests.exceptions.ConnectionError as errc:
                print(f'HTTP request connection error when posting Update request.')
                print(f'Error: {errc}')
                print(f'Firmware upgrading failed.')
                sys.exit()
            except requests.exceptions.Timeout as errt:
                print(f'HTTP request timeout when posting Update request.')
                print(f'Error: {errt}')
                print(f'Firmware upgrading failed.')
                sys.exit()
            except requests.exceptions.RequestException as err:
                print(f'HTTP request return a connection Error when posting Update request.')
                print(f'Error: {err}')
                print(f'Firmware upgrading failed.')
                sys.exit()

            if r.status_code == 200:
                resp = r.text
            else:
                resp = None
            # When the json file is not yet created, get() returns 0.
            if (resp is None) or (resp == '0'):
                continue

            try:
                request_answer = json.loads(resp)
            except Exception as e:
                print(f'Failed to parse the answer "{resp}". {e}')
                continue

            if request_answer.get('STATUS'):
                status_code = int(request_answer.get('STATUS').get('Code'))
                status_msg = request_answer.get('STATUS').get('MSG')

            if status_code in [0, 1]:
                keys = sorted(request_answer.get('LOG').keys())
                if keys:
                    last_progress = progress
                    progress = request_answer.get('LOG').get(keys[-1])
                    new_progress = progress.replace(last_progress, '')
                    if '#' in new_progress:
                        print(f' ', end='')
                    print(f'{new_progress}', end='', flush='True')
                    if '100%' in new_progress:
                        print(f'')
                if status_code == 0:
                    update_done = True
                    print(f'{status_msg}')
            else:
                print(f'{status_msg}')
                raise RuntimeError('Error while updating')

            time.sleep(2)

        print(f'Update done')
        time.sleep(5)
        print(f'Rebooting ...', end='', flush='True')
        while(not robot_inst.connect()):
            time.sleep(1)
            print(f'...', end='', flush='True')
        print(f'\nRebooting done.')
        robot_inst.disconnect()


if __name__ == '__main__':
    """Update the robot firmware.

    """
    [robot_fw_path, robot_ip_address] = get_args()
    update_robot(robot_fw_path, robot_ip_address)
