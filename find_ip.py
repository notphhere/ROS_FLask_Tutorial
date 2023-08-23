import subprocess


def get_local_ipv4_addresses():
    try:
        # Run the "hostname -I" command and capture its output
        result = subprocess.run(
            ["hostname", "-I"], stdout=subprocess.PIPE, text=True)
        output = result.stdout.strip()
        # Split the output by spaces to get individual IPv4 addresses
        ipv4_addresses = output.split()
        return ipv4_addresses
    except Exception as e:
        print("Error:", e)
        return None


if __name__ == "__main__":
    local_ipv4_addresses = get_local_ipv4_addresses()[0]
    if local_ipv4_addresses:
        print("Local IPv4 Addresses:", local_ipv4_addresses)
    else:
        print("Failed to retrieve local IPv4 addresses.")
