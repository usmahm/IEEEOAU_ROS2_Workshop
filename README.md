# 🐢 ROS 2 Setup on Windows (with WSL2 + Ubuntu 22.04)

This guide walks you through setting up **ROS 2 Humble** on Windows using **Windows Subsystem for Linux (WSL2)**.  
Follow each step carefully. Screenshots are included to help visualize the process.

---

## 1. Enable WSL and Virtualization  

1. Open **Control Panel**  
![1](https://github.com/user-attachments/assets/9df7cc34-950a-4391-adf9-32799a9e5917)

2. Click on **Programs**  
![2](https://github.com/user-attachments/assets/282b822a-1616-4363-97b5-484450edc830)

3. Select **Turn Windows features on or off**  
![3](https://github.com/user-attachments/assets/42a43b25-c0d6-4927-98f2-1e769e61841d)

4. Tick both:  
   - **Virtual Machine Platform**  
   - **Windows Subsystem for Linux**  
![4](https://github.com/user-attachments/assets/11d4ba8a-c632-42cb-ae50-4a139687461e)

5. Click **OK** and then **Restart now**  
![5](https://github.com/user-attachments/assets/34aabec3-1a2a-4078-ba5d-ed1633e303da)

---

## 2. Install WSL  

1. Open **Windows PowerShell** as Administrator  
![6](https://github.com/user-attachments/assets/402ed6ef-bbe9-40d2-9c91-5ee4869ff126)

2. Run the following command:  

   ```powershell
   wsl --install
   ```  

![7](https://github.com/user-attachments/assets/9063ceba-7907-4eb2-a6c6-57522e18ba12)

3. Create a default **Unix username and password** when prompted.  
<img width="1366" height="768" alt="8" src="https://github.com/user-attachments/assets/27f3d129-2701-4efd-ac28-598ffeab3ab7" />

⚠️ Don’t forget this password!  

5. Close and reopen PowerShell.  

6. Install Ubuntu 22.04:  

   ```powershell
   wsl --install -d Ubuntu-22.04
   ```  
![10](https://github.com/user-attachments/assets/bda9da7e-e0d0-49cc-898f-25740cb17810)

6. If prompted, create a new username and password for Ubuntu.  

7. Close PowerShell, search for **Ubuntu 22.04** in the start menu, and open it.  
![12](https://github.com/user-attachments/assets/8177f1b4-2e4e-494c-939f-e66c16c26152)

---

## 3. Configure Locale 

Run the commands that follow (line by line) inside the Ubuntu terminal opened in the last step


```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
```

---

## 4. Add ROS 2 Repositories  

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

Update and upgrade:  

```bash
sudo apt update
sudo apt upgrade
```

---

## 5. Install ROS 2 Humble  

```bash
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```

Set up the environment:  

```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

---

## 6. Test Installation  

1. Open a new Ubuntu terminal.  
2. Run the ROS 2 demo:  

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

3. In another terminal, run:  

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

👉 If the talker publishes and the listener receives messages, your setup works! 🎉  

<img width="1366" height="768" alt="13" src="https://github.com/user-attachments/assets/96086fce-dff3-4bf9-b3a3-b1f943e8fd71" />
<img width="1366" height="768" alt="14" src="https://github.com/user-attachments/assets/3ece7c73-c0aa-4d2e-9eff-9e396434eabe" />

---

## 7. Install VS Code  

1. Download and install **Visual Studio Code** from [here](https://code.visualstudio.com/download/).  
2. Close the Ubuntu terminal, reopen it, then run:  

```bash
code .
```

This will open VS Code directly from your Ubuntu environment.  

---

## ✅ You’re Ready!  

You now have:  
- WSL2 with Ubuntu 22.04  
- ROS 2 Humble fully installed  
- VS Code integrated with your ROS environment  

---

## 🔑 Forgot Your Ubuntu Password?  

If you forget the username or password you created during setup, you can reset it:

1. Open **Command Prompt** and list installed WSL distributions:  

   ```powershell
   wsl -l
   ```

   Example output:  
   ```
   Windows Subsystem for Linux Distributions:
   Ubuntu-22.04 (Default)
   ```

2. Set the default user for Ubuntu to **root**:  

   ```powershell
   ubuntu2204 config --default-user root
   ```

3. Open a new Ubuntu terminal (from Start Menu).  
   You will now be logged in as `root`.

4. Check your current user and find available usernames:  

   ```bash
   whoami          # shows current user (root)
   ls /home        # lists all user home directories
   ```

5. Reset your user’s password (replace `your_username` with the name from step 4):  

   ```bash
   passwd your_username
   ```

6. Set your preferred new password.

7. (Optional but recommended) Switch the default user back to your normal account:  

   ```powershell
   ubuntu2204 config --default-user your_username
   ```


## 👥 Contributors  

Thanks to the following contributors for helping with setup, testing, and documentation:  

- [Ahmad Usman](https://github.com/usmahm)
- Mujeeb Opabode
- [Jelili-Ibrahim Abdul-Azeez](https://github.com/Azeez-Ibrahim)  

---
