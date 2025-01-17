import algotourrouter_pb2
import algotourrouter_pb2_grpc

import grpc
from grpc_status import rpc_status
import firebase_admin
from firebase_admin import credentials, firestore
from google.cloud.firestore_v1.base_query import FieldFilter
from google.cloud.firestore_v1._helpers import GeoPoint


cred = credentials.Certificate("./firebase.json")
firebase_admin.initialize_app(cred)
db = firestore.client()

# Function to process new document
def process_new_document(doc_data: dict, ref):
    print("Processing new document:")
    ref.update({
          'state': 'processing'
      })
    print(doc_data)



    with grpc.insecure_channel("localhost:50051") as channel:
      stub = algotourrouter_pb2_grpc.AlgorouterStub(channel)
      rrrpc = algotourrouter_pb2.RouteRequestRPC(startLon=doc_data['startpoint'].longitude, startLat=doc_data['startpoint'].latitude, endLon=doc_data['endpoint'].longitude, endLat=doc_data['endpoint'].latitude)
      
      try:
        path_future = stub.DoRouting.future(rrrpc, timeout=10)


        res = path_future.result()
        print(outpath := res.pathFile)


        with open(outpath) as o:
          print('writing ref')
          r = o.read()
          ref.update({
              'route': r,
              'state': 'processed'
          })
      except grpc.RpcError as rpc_error:
        status = rpc_status.from_call(rpc_error)
        print("status failed:", status)
        ref.update({
              'state': 'failed'
          })

      print('request closed')
         

# Callback function to handle changes
def on_snapshot(col_snapshot, changes, read_time):
    print(f"Snapshot received at {read_time}")
    for change in changes:
        if change.type.name == 'ADDED':
            print(f"[Firestore] New document: {change.document.id}")
            # Get the new document's data
            doc_data = change.document.to_dict()
            # Call the processing function with the new document's data
            
            nd = process_new_document(doc_data, change.document.reference)
            print("ERROR processing doc", change.document.reference)

        elif change.type.name == 'MODIFIED':
            print(f"[Firestore] Modified document: {change.document.id}")
        elif change.type.name == 'REMOVED':
            print(f"[Firestore] Removed document: {change.document.id}")


collection_ref = db.collection('tours')
col_query_watch = collection_ref.where("state", '==', 'waiting').on_snapshot(on_snapshot)

import time
while True:
    time.sleep(1)