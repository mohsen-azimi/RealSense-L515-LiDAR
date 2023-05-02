import subprocess
import socket


def git_push():
    hostname = socket.gethostname()
    if hostname == "APC":
        subprocess.run(["git", "add", "*.py"])
        # subprocess.run(["git", "add", "sub_mask_2d.py"])
        subprocess.run(["git", "add", "*.md"])
        # subprocess.run(["git", "add", "*.txt"])
        subprocess.run(["git", "commit", "-m", "updated"])
        subprocess.run(["git", "push"])
        print(f"Changes Pushed from {hostname} to GitHub")
    else:
        print("Code not running on Mohsen's Laptop, GitHub is not updated")


if __name__ == '__main__':
    git_push()
