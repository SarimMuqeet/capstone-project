#Note: need to setup SSH keys on Jetson-Nano and Jackal

import paramiko

JACKAL1_IP = "129.97.71.36"
JACKAL2_IP = "129.97.71.37"

# JACKAL_IP = "192.168.X.X" 
JACKAL_IP = JACKAL2_IP
USERNAME = "administrator"
#SSH keygen path on Jetson Nano
SSH_KEY_PATH = "/home/homey/.ssh/id_rsa"

def send_ssh_command(command):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(JACKAL_IP, username=USERNAME, key_filename=SSH_KEY_PATH)

    stdin, stdout, stderr = ssh.exec_command(command)
    print(stdout.read().decode())
    ssh.close()

#Command to move Jackal
send_ssh_command("./move_rot_x.sh 0.2 0") 