#!/usr/bin/env python3
"""
Simple WebSocket test script to verify connectivity and debug connection issues.
"""

import time
import threading

# Try to import websocket library
try:
    import websocket
    WEBSOCKET_AVAILABLE = True
    print("✓ WebSocket library available")
except ImportError:
    print("✗ WebSocket library not available. Install with: pip install websocket-client")
    WEBSOCKET_AVAILABLE = False
    exit(1)

def test_websocket_connection(url, timeout=5):
    """Test WebSocket connection with detailed error reporting."""
    print(f"\nTesting connection to: {url}")
    print(f"Timeout: {timeout} seconds")
    
    try:
        print("Attempting to connect...")
        start_time = time.time()
        
        # Create connection
        ws = websocket.create_connection(url, timeout=timeout)
        
        connect_time = time.time() - start_time
        print(f"✓ Connected successfully in {connect_time:.2f} seconds")
        
        # Test ping
        try:
            ws.ping()
            print("✓ Ping successful")
        except Exception as e:
            print(f"✗ Ping failed: {e}")
        
        # Test send/receive
        try:
            test_message = "test_message"
            ws.send(test_message)
            print(f"✓ Sent message: {test_message}")
            
            # Try to receive (with timeout)
            ws.settimeout(2)
            try:
                response = ws.recv()
                print(f"✓ Received response: {response}")
            except websocket.WebSocketTimeoutException:
                print("ℹ No response received (timeout) - this is normal for some servers")
            except Exception as e:
                print(f"ℹ Receive error (expected for test): {e}")
                
        except Exception as e:
            print(f"✗ Send failed: {e}")
        
        # Close connection
        ws.close()
        print("✓ Connection closed cleanly")
        return True
        
    except websocket.WebSocketConnectionClosedException as e:
        print(f"✗ Connection closed: {e}")
        return False
    except websocket.WebSocketBadStatusException as e:
        print(f"✗ Bad status: {e}")
        return False
    except websocket.WebSocketTimeoutException as e:
        print(f"✗ Connection timeout: {e}")
        return False
    except OSError as e:
        if "Connection refused" in str(e):
            print(f"✗ Connection refused - server may not be running at {url}")
        elif "No route to host" in str(e):
            print(f"✗ No route to host - check network connectivity to {url}")
        else:
            print(f"✗ OS Error: {e}")
        return False
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        return False

def test_websocket_server(url):
    """Test if WebSocket server is responding."""
    print(f"\nTesting WebSocket server at: {url}")
    
    # Test basic connection
    if test_websocket_connection(url, timeout=10):
        print("\n✓ WebSocket server is working correctly!")
        return True
    else:
        print("\n✗ WebSocket server test failed!")
        return False

def main():
    print("WebSocket Connection Test Tool")
    print("=" * 40)
    
    # Test URLs
    test_urls = [
        "ws://frodo.local:8765",
        "ws://localhost:8765",
        "ws://127.0.0.1:8765"
    ]
    
    print("\nTesting common WebSocket URLs...")
    
    working_urls = []
    for url in test_urls:
        if test_websocket_server(url):
            working_urls.append(url)
    
    if working_urls:
        print(f"\n✓ Working WebSocket URLs: {', '.join(working_urls)}")
        print(f"Use one of these URLs in your main script.")
    else:
        print("\n✗ No working WebSocket URLs found.")
        print("\nTroubleshooting tips:")
        print("1. Make sure your WebSocket server is running")
        print("2. Check if the port is correct (default: 8765)")
        print("3. Verify network connectivity")
        print("4. Check firewall settings")
        print("5. Try using 'localhost' instead of hostname")

if __name__ == "__main__":
    main() 