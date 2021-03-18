using DeepLearning;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace SIGGRAPH_2017 {
	public class BioAnimation_Original : MonoBehaviour {

		public bool Inspect = false;

		public float TargetBlending = 0.25f;
		public float GaitTransition = 0.25f;
		public float TrajectoryCorrection = 1f;

		public Controller Controller;

		private Actor Actor;
		private PFNN NN;
		private Trajectory Trajectory;

		private Vector3 TargetDirection;
		private Vector3 TargetVelocity;

		//Rescaling for character (cm to m)
		private float UnitScale = 100f;

		//State
		private Vector3[] Positions = new Vector3[0];
		private Vector3[] Forwards = new Vector3[0];
		private Vector3[] Ups = new Vector3[0];
		private Vector3[] Velocities = new Vector3[0];

		//Trajectory for 60 Hz framerate
		private const int PointSamples = 12;
		private const int RootSampleIndex = 6;
		private const int RootPointIndex = 60; //当前点是第7个点，前6个是历史点，因为密度是10，第七个点在60 index处
		private const int FuturePoints = 5;
		private const int PreviousPoints = 6;
		private const int PointDensity = 10;

		void Reset() {
			Controller = new Controller();
		}

		void Awake() {
			Actor = GetComponent<Actor>();
			NN = GetComponent<PFNN>();
			TargetDirection = new Vector3(transform.forward.x, 0f, transform.forward.z);
			TargetVelocity = Vector3.zero;
			Positions = new Vector3[Actor.Bones.Length];
			Forwards = new Vector3[Actor.Bones.Length];
			Ups = new Vector3[Actor.Bones.Length];
			Velocities = new Vector3[Actor.Bones.Length];
			// 12个点，每2个点中间有9个，一共111个
			Trajectory = new Trajectory(111, Controller.GetNames(), transform.position, TargetDirection);
			Trajectory.Postprocess();
			if(Controller.Styles.Length > 0) {
				for(int i=0; i<Trajectory.Points.Length; i++) {
					Trajectory.Points[i].Styles[0] = 1f;
				}
			}
			for(int i=0; i<Actor.Bones.Length; i++) {
				Positions[i] = Actor.Bones[i].Transform.position;
				Forwards[i] = Actor.Bones[i].Transform.forward;
				Ups[i] = Actor.Bones[i].Transform.up;
				Velocities[i] = Vector3.zero;
			}

			if(NN.Parameters == null) {
				Debug.Log("No parameters saved.");
				return;
			}
			NN.LoadParameters();
		}

		void Start() {
			Utility.SetFPS(60);
		}

		void Update() {
			if(NN.Parameters == null) {
				return;
			}

			//Update Target Direction / Velocity， 从控制器可以获得面朝向，移动方向
			TargetDirection = Vector3.Lerp(TargetDirection, Quaternion.AngleAxis(Controller.QueryTurn()*60f, Vector3.up) * Trajectory.Points[RootPointIndex].GetDirection(), TargetBlending);
			TargetVelocity = Vector3.Lerp(TargetVelocity, (Quaternion.LookRotation(TargetDirection, Vector3.up) * Controller.QueryMove()).normalized, TargetBlending);

			//Update Gait， ， 从控制器获得姿势，这个是float，因为值得改变是每帧插值过去的
			for (int i=0; i<Controller.Styles.Length; i++) {
				Trajectory.Points[RootPointIndex].Styles[i] = Utility.Interpolate(Trajectory.Points[RootPointIndex].Styles[i], Controller.Styles[i].Query() ? 1f : 0f, GaitTransition);
			}
			//For Human Only
			//Trajectory.Points[RootPointIndex].Styles[0] = Utility.Interpolate(Trajectory.Points[RootPointIndex].Styles[0], 1.0f - Mathf.Clamp(Vector3.Magnitude(TargetVelocity) / 0.1f, 0.0f, 1.0f), GaitTransition);
			//Trajectory.Points[RootPointIndex].Styles[1] = Mathf.Max(Trajectory.Points[RootPointIndex].Styles[1] - Trajectory.Points[RootPointIndex].Styles[2], 0f);
			//

			/*
			//Blend Trajectory Offset
			Vector3 positionOffset = transform.position - Trajectory.Points[RootPointIndex].GetPosition();
			Quaternion rotationOffset = Quaternion.Inverse(Trajectory.Points[RootPointIndex].GetRotation()) * transform.rotation;
			Trajectory.Points[RootPointIndex].SetPosition(Trajectory.Points[RootPointIndex].GetPosition() + positionOffset);
			Trajectory.Points[RootPointIndex].SetDirection(rotationOffset * Trajectory.Points[RootPointIndex].GetDirection());

			for(int i=RootPointIndex; i<Trajectory.Points.Length; i++) {
				float factor = 1f - (i - RootPointIndex)/(RootPointIndex - 1f);
				Trajectory.Points[i].SetPosition(Trajectory.Points[i].GetPosition() + factor*positionOffset);
			}
			*/

			//Predict Future Trajectory
			Vector3[] trajectory_positions_blend = new Vector3[Trajectory.Points.Length];
			trajectory_positions_blend[RootPointIndex] = Trajectory.Points[RootPointIndex].GetPosition();

			for(int i=RootPointIndex+1; i<Trajectory.Points.Length; i++) {
				float bias_pos = 0.75f;
				float bias_dir = 1.25f;
				float scale_pos = (1.0f - Mathf.Pow(1.0f - ((float)(i - RootPointIndex) / (RootPointIndex)), bias_pos));
				float scale_dir = (1.0f - Mathf.Pow(1.0f - ((float)(i - RootPointIndex) / (RootPointIndex)), bias_dir));
				float vel_boost = PoolBias(); //基本就是1啊

				// 因为总共预测得是1秒之后位置，所以每个点相对偏移要乘以这个
				float rescale = 1f / (Trajectory.Points.Length - (RootPointIndex + 1f)); 

				// 未来位置相对偏移，是预测位置相对偏移，和当前移动速度 的一个插值
				trajectory_positions_blend[i] = trajectory_positions_blend[i-1] + Vector3.Lerp(
					Trajectory.Points[i].GetPosition() - Trajectory.Points[i-1].GetPosition(), 
					vel_boost * rescale * TargetVelocity,
					scale_pos);


				// 未来方向 是预测方向 和 当前方向 的插值
				Trajectory.Points[i].SetDirection(Vector3.Lerp(Trajectory.Points[i].GetDirection(), TargetDirection, scale_dir));

				// 未来姿势gait是跟当前一致
				for(int j=0; j<Trajectory.Points[i].Styles.Length; j++) {
					Trajectory.Points[i].Styles[j] = Trajectory.Points[RootPointIndex].Styles[j];
				}
			}
			
			// 设置上位置
			for(int i=RootPointIndex+1; i<Trajectory.Points.Length; i++) {
				Trajectory.Points[i].SetPosition(trajectory_positions_blend[i]);
			}

			// 只有整点 来做地形处理，获得高度啥的
			for(int i=RootPointIndex; i<Trajectory.Points.Length; i+=PointDensity) {
				Trajectory.Points[i].Postprocess();
			}

			// 其他中间点，都通过整点来插值得到
			for(int i=RootPointIndex+1; i<Trajectory.Points.Length; i++) {
				//ROOT	1		2		3		4		5
				//.x....x.......x.......x.......x.......x
				Trajectory.Point prev = GetPreviousSample(i); // 得到前一个整点
				Trajectory.Point next = GetNextSample(i);     // 下一个整点
				float factor = (float)(i % PointDensity) / PointDensity;

				Trajectory.Points[i].SetPosition((1f-factor)*prev.GetPosition() + factor*next.GetPosition());
				Trajectory.Points[i].SetDirection((1f-factor)*prev.GetDirection() + factor*next.GetDirection());
				Trajectory.Points[i].SetLeftsample((1f-factor)*prev.GetLeftSample() + factor*next.GetLeftSample());
				Trajectory.Points[i].SetRightSample((1f-factor)*prev.GetRightSample() + factor*next.GetRightSample());
				Trajectory.Points[i].SetSlope((1f-factor)*prev.GetSlope() + factor*next.GetSlope());
			}

			//Avoid Collisions，跟障碍物Obstacles碰撞后，后面预测的点都维持在碰撞的当前点上
			CollisionChecks(RootPointIndex+1);

			if(NN.Parameters != null) {
				//Calculate Root
				Matrix4x4 currentRoot = Trajectory.Points[RootPointIndex].GetTransformation();
				Matrix4x4 previousRoot = Trajectory.Points[RootPointIndex-1].GetTransformation();
					
				//Input Trajectory Positions / Directions
				for(int i=0; i<PointSamples; i++) {
					Vector3 pos = Trajectory.Points[i*PointDensity].GetPosition().GetRelativePositionTo(currentRoot);
					Vector3 dir = Trajectory.Points[i*PointDensity].GetDirection().GetRelativeDirectionTo(currentRoot);
					NN.SetInput(PointSamples*0 + i, UnitScale * pos.x); // 神经网络输入：先是12个x位置（相对于当前坐标）
					NN.SetInput(PointSamples*1 + i, UnitScale * pos.z); // 12个z位置
					NN.SetInput(PointSamples*2 + i, dir.x);             // 12个x方向
					NN.SetInput(PointSamples*3 + i, dir.z);             // 12个z方向
				}

				//Input Trajectory Gaits
				for (int i=0; i<PointSamples; i++) {                    // 12个stand 姿势值， 12个 walk姿势值，12个 jog姿势值，crouch，jump，bump，共12*6
					for(int j=0; j<Trajectory.Points[i*PointDensity].Styles.Length; j++) {
						NN.SetInput(PointSamples*(4+j) + i, Trajectory.Points[i*PointDensity].Styles[j]);
					}
					//FOR HUMAN ONLY， 这是把jump的gait信息给覆盖了。破面太陡，就自动jump
					NN.SetInput(PointSamples*8 + i, Trajectory.Points[i*PointDensity].GetSlope());
					//
				}

				//Input Previous Bone Positions / Velocities
				for(int i=0; i<Actor.Bones.Length; i++) {
					int o = 10*PointSamples;											// input的index从120开始了
					Vector3 pos = Positions[i].GetRelativePositionTo(previousRoot);
					Vector3 vel = Velocities[i].GetRelativeDirectionTo(previousRoot);
					NN.SetInput(o + Actor.Bones.Length*3*0 + i*3+0, UnitScale * pos.x); // 31个骨骼节点位置信息，x,y,z
					NN.SetInput(o + Actor.Bones.Length*3*0 + i*3+1, UnitScale * pos.y);
					NN.SetInput(o + Actor.Bones.Length*3*0 + i*3+2, UnitScale * pos.z);
					NN.SetInput(o + Actor.Bones.Length*3*1 + i*3+0, UnitScale * vel.x); // 31个骨骼节点速度信息，x,y,z
					NN.SetInput(o + Actor.Bones.Length*3*1 + i*3+1, UnitScale * vel.y);
					NN.SetInput(o + Actor.Bones.Length*3*1 + i*3+2, UnitScale * vel.z);
				}

				//Input Trajectory Heights
				for(int i=0; i<PointSamples; i++) {
					int o = 10*PointSamples + Actor.Bones.Length*3*2;                   // input的index从120+186=306开始了
					NN.SetInput(o + PointSamples*0 + i, UnitScale * (Trajectory.Points[i*PointDensity].GetRightSample().y - currentRoot.GetPosition().y));	// 12个右边点高度
					NN.SetInput(o + PointSamples*1 + i, UnitScale * (Trajectory.Points[i*PointDensity].GetPosition().y - currentRoot.GetPosition().y));     // 12个中间点高度
					NN.SetInput(o + PointSamples*2 + i, UnitScale * (Trajectory.Points[i*PointDensity].GetLeftSample().y - currentRoot.GetPosition().y));   // 12个左边点高度
				}

				//Predict
				float rest = Mathf.Pow(1.0f-Trajectory.Points[RootPointIndex].Styles[0], 0.25f); // stand为1时，rest为0，阻尼damping为0.9
				NN.SetDamping(1f - (rest * 0.9f + 0.1f)); // stand为0时，rest为1，damping为0，从而下帧phase就取 当前phase + 此帧predict后的phase偏移，实际的PhaseIndex=3
				NN.Predict();

				//Update Past Trajectory,之前的[1,60]帧记录下来到[0,59]
				for (int i=0; i<RootPointIndex; i++) { 
					Trajectory.Points[i].SetPosition(Trajectory.Points[i+1].GetPosition());
					Trajectory.Points[i].SetDirection(Trajectory.Points[i+1].GetDirection());
					Trajectory.Points[i].SetLeftsample(Trajectory.Points[i+1].GetLeftSample());
					Trajectory.Points[i].SetRightSample(Trajectory.Points[i+1].GetRightSample());
					Trajectory.Points[i].SetSlope(Trajectory.Points[i+1].GetSlope());
					for(int j=0; j<Trajectory.Points[i].Styles.Length; j++) {
						Trajectory.Points[i].Styles[j] = Trajectory.Points[i+1].Styles[j];
					}
				}

				//Update Current Trajectory，Y的 0,1是预测的下一帧的dx，dz，2是预测的下一帧的转向角度
				Trajectory.Points[RootPointIndex].SetPosition((rest * new Vector3(NN.GetOutput(0) / UnitScale, 0f, NN.GetOutput(1) / UnitScale)).GetRelativePositionFrom(currentRoot));
				Trajectory.Points[RootPointIndex].SetDirection(Quaternion.AngleAxis(rest * Mathf.Rad2Deg * (-NN.GetOutput(2)), Vector3.up) * Trajectory.Points[RootPointIndex].GetDirection());
				Trajectory.Points[RootPointIndex].Postprocess();
				Matrix4x4 nextRoot = Trajectory.Points[RootPointIndex].GetTransformation();

				//Update Future Trajectory，剩下的路径点，用当前帧的路径点 + 本位点预测相对于 实际设置的位置和方向的偏移
				for(int i=RootPointIndex+1; i<Trajectory.Points.Length; i++) {
					Trajectory.Points[i].SetPosition(Trajectory.Points[i].GetPosition() + (rest * new Vector3(NN.GetOutput(0) / UnitScale, 0f, NN.GetOutput(1) / UnitScale)).GetRelativeDirectionFrom(nextRoot));
				}
				for(int i=RootPointIndex+1; i<Trajectory.Points.Length; i++) {
					int w = RootSampleIndex;
					float m = Mathf.Repeat(((float)i - (float)RootPointIndex) / (float)PointDensity, 1.0f);                         // index从8开始
					float posX = (1-m) * NN.GetOutput(8+(w*0)+(i/PointDensity)-w) + m * NN.GetOutput(8+(w*0)+(i/PointDensity)-w+1); // 先有6个posX
					float posZ = (1-m) * NN.GetOutput(8+(w*1)+(i/PointDensity)-w) + m * NN.GetOutput(8+(w*1)+(i/PointDensity)-w+1); // 6个posZ
					float dirX = (1-m) * NN.GetOutput(8+(w*2)+(i/PointDensity)-w) + m * NN.GetOutput(8+(w*2)+(i/PointDensity)-w+1); // 6个dirX
					float dirZ = (1-m) * NN.GetOutput(8+(w*3)+(i/PointDensity)-w) + m * NN.GetOutput(8+(w*3)+(i/PointDensity)-w+1); // 6个dirZ
					Trajectory.Points[i].SetPosition( //这50个中间的45个点的pos，dir好像白计算了，后面会从12个位点中做插值
						Utility.Interpolate(
							Trajectory.Points[i].GetPosition(),
							new Vector3(posX / UnitScale, 0f, posZ / UnitScale).GetRelativePositionFrom(nextRoot),
							TrajectoryCorrection //实际设置位0.75
							)
						);
					Trajectory.Points[i].SetDirection(
						Utility.Interpolate(
							Trajectory.Points[i].GetDirection(),
							new Vector3(dirX, 0f, dirZ).normalized.GetRelativeDirectionFrom(nextRoot),
							TrajectoryCorrection
							)
						);
				}

				for(int i=RootPointIndex+PointDensity; i<Trajectory.Points.Length; i+=PointDensity) {
					Trajectory.Points[i].Postprocess();
				}

				for(int i=RootPointIndex+1; i<Trajectory.Points.Length; i++) {
					//ROOT	1		2		3		4		5
					//.x....x.......x.......x.......x.......x
					Trajectory.Point prev = GetPreviousSample(i);
					Trajectory.Point next = GetNextSample(i);
					float factor = (float)(i % PointDensity) / PointDensity;

					Trajectory.Points[i].SetPosition((1f-factor)*prev.GetPosition() + factor*next.GetPosition());
					Trajectory.Points[i].SetDirection((1f-factor)*prev.GetDirection() + factor*next.GetDirection());
					Trajectory.Points[i].SetLeftsample((1f-factor)*prev.GetLeftSample() + factor*next.GetLeftSample());
					Trajectory.Points[i].SetRightSample((1f-factor)*prev.GetRightSample() + factor*next.GetRightSample());
					Trajectory.Points[i].SetSlope((1f-factor)*prev.GetSlope() + factor*next.GetSlope());
				}

				//Avoid Collisions
				CollisionChecks(RootPointIndex);
				
				//Compute Posture
				int opos = 8 + 4*RootSampleIndex + Actor.Bones.Length*3*0;    // index从 8+24=30开始，有31*3个骨骼点位置
				int ovel = 8 + 4*RootSampleIndex + Actor.Bones.Length*3*1;    // 从30+31*3开始，有93个骨骼点速度
				//int orot = 8 + 4*RootSampleIndex + Actor.Bones.Length*3*2;
				for(int i=0; i<Actor.Bones.Length; i++) {			
					Vector3 position = new Vector3(NN.GetOutput(opos+i*3+0), NN.GetOutput(opos+i*3+1), NN.GetOutput(opos+i*3+2)) / UnitScale;
					Vector3 velocity = new Vector3(NN.GetOutput(ovel+i*3+0), NN.GetOutput(ovel+i*3+1), NN.GetOutput(ovel+i*3+2)) / UnitScale;
					//Quaternion rotation = new Quaternion(PFNN.GetOutput(orot+i*3+0), PFNN.GetOutput(orot+i*3+1), PFNN.GetOutput(orot+i*3+2), 0f).Exp();
					Positions[i] = Vector3.Lerp(Positions[i].GetRelativePositionTo(currentRoot) + velocity, position, 0.5f).GetRelativePositionFrom(currentRoot);
					Velocities[i] = velocity.GetRelativeDirectionFrom(currentRoot);
					//rotations[i] = rotation.GetRelativeRotationFrom(currentRoot);
				}
				
				//Update Posture
				transform.position = nextRoot.GetPosition();
				transform.rotation = nextRoot.GetRotation();
				for(int i=0; i<Actor.Bones.Length; i++) {
					Actor.Bones[i].Transform.position = Positions[i]; //也就是说这个允许各个拉伸，但没有任何旋转，这应该是它跟Adam的区别。
					Actor.Bones[i].Transform.rotation = Quaternion.LookRotation(Forwards[i], Ups[i]);
				}
			}
		}

		private float PoolBias() {
			float[] styles = Trajectory.Points[RootPointIndex].Styles;
			float bias = 0f;
			for(int i=0; i<styles.Length; i++) {
				float _bias = Controller.Styles[i].Bias; // 这个配置都是1
				float max = 0f;
				for(int j=0; j<Controller.Styles[i].Multipliers.Length; j++) {
					if(Input.GetKey(Controller.Styles[i].Multipliers[j].Key)) {
						max = Mathf.Max(max, Controller.Styles[i].Bias * Controller.Styles[i].Multipliers[j].Value);
					}
				}
				for(int j=0; j<Controller.Styles[i].Multipliers.Length; j++) {
					if(Input.GetKey(Controller.Styles[i].Multipliers[j].Key)) {
						_bias = Mathf.Min(max, _bias * Controller.Styles[i].Multipliers[j].Value);
					}
				}
				bias += styles[i] * _bias;
			}
			return bias;
		}

		private Trajectory.Point GetSample(int index) {
			return Trajectory.Points[Mathf.Clamp(index*10, 0, Trajectory.Points.Length-1)];
		}

		private Trajectory.Point GetPreviousSample(int index) {
			return GetSample(index / 10);
		}

		private Trajectory.Point GetNextSample(int index) {
			if(index % 10 == 0) {
				return GetSample(index / 10);
			} else {
				return GetSample(index / 10 + 1);
			}
		}

		// 跟障碍物Obstacles碰撞后，后面预测的点都维持在当前点上。
		private void CollisionChecks(int start) {
			for(int i=start; i<Trajectory.Points.Length; i++) {
				float safety = 0.5f;
				Vector3 previousPos = Trajectory.Points[i-1].GetPosition();
				Vector3 currentPos = Trajectory.Points[i].GetPosition();
				Vector3 testPos = previousPos + safety*(currentPos-previousPos).normalized;
				Vector3 projectedPos = Utility.ProjectCollision(previousPos, testPos, LayerMask.GetMask("Obstacles"));
				if(testPos != projectedPos) {
					Vector3 correctedPos = testPos + safety * (previousPos-testPos).normalized; //这就是previousPos吧
					Trajectory.Points[i].SetPosition(correctedPos);
				}
			}
		}

		void OnGUI() {
			GUI.color = UltiDraw.Mustard;
			GUI.backgroundColor = UltiDraw.Black;
			float height = 0.05f;
			GUI.Box(Utility.GetGUIRect(0.025f, 0.05f, 0.3f, Controller.Styles.Length*height), "");
			for(int i=0; i<Controller.Styles.Length; i++) {
				GUI.Label(Utility.GetGUIRect(0.05f, 0.075f + i*0.05f, 0.025f, height), Controller.Styles[i].Name);
				string keys = string.Empty;
				for(int j=0; j<Controller.Styles[i].Keys.Length; j++) {
					keys += Controller.Styles[i].Keys[j].ToString() + " ";
				}
				GUI.Label(Utility.GetGUIRect(0.075f, 0.075f + i*0.05f, 0.05f, height), keys);
				GUI.HorizontalSlider(Utility.GetGUIRect(0.125f, 0.075f + i*0.05f, 0.15f, height), Trajectory.Points[RootPointIndex].Styles[i], 0f, 1f);
			}
		}

		void OnRenderObject() {
			/*
			UltiDraw.Begin();
			UltiDraw.DrawGUICircle(new Vector2(0.5f, 0.85f), 0.075f, UltiDraw.Black.Transparent(0.5f));
			Quaternion rotation = Quaternion.AngleAxis(-360f * NN.GetPhase() / (2f * Mathf.PI), Vector3.forward);
			Vector2 a = rotation * new Vector2(-0.005f, 0f);
			Vector2 b = rotation *new Vector3(0.005f, 0f);
			Vector3 c = rotation * new Vector3(0f, 0.075f);
			UltiDraw.DrawGUITriangle(new Vector2(0.5f + b.x/Screen.width*Screen.height, 0.85f + b.y), new Vector2(0.5f + a.x/Screen.width*Screen.height, 0.85f + a.y), new Vector2(0.5f + c.x/Screen.width*Screen.height, 0.85f + c.y), UltiDraw.Cyan);
			UltiDraw.End();
			*/

			if(Application.isPlaying) {
				if(NN.Parameters == null) {
					return;
				}

				UltiDraw.Begin();
				// 红色是目标面朝方向
				UltiDraw.DrawLine(Trajectory.Points[RootPointIndex].GetPosition(), Trajectory.Points[RootPointIndex].GetPosition() + TargetDirection, 0.05f, 0f, UltiDraw.Red.Transparent(0.75f));
				// 绿色是目标移动方向
				UltiDraw.DrawLine(Trajectory.Points[RootPointIndex].GetPosition(), Trajectory.Points[RootPointIndex].GetPosition() + TargetVelocity, 0.05f, 0f, UltiDraw.Green.Transparent(0.75f));
				UltiDraw.End();

				// 画地面轨迹
				Trajectory.Draw(10);
				
				UltiDraw.Begin();
				for(int i=0; i<Actor.Bones.Length; i++) {
					// 紫色是画 骨骼速度
					UltiDraw.DrawArrow(
						Actor.Bones[i].Transform.position,
						Actor.Bones[i].Transform.position + Velocities[i],
						0.75f,
						0.0075f,
						0.05f,
						UltiDraw.Purple.Transparent(0.5f)
					);
				}
				UltiDraw.End();
			}
		}

		void OnDrawGizmos() {
			if(!Application.isPlaying) {
				OnRenderObject();
			}
		}
	}

	#if UNITY_EDITOR
	[CustomEditor(typeof(BioAnimation_Original))]
	public class BioAnimation_Original_Editor : Editor {

			public BioAnimation_Original Target;

			void Awake() {
				Target = (BioAnimation_Original)target;
			}

			public override void OnInspectorGUI() {
				Undo.RecordObject(Target, Target.name);

				Inspector();
				Target.Controller.Inspector();

				if(GUI.changed) {
					EditorUtility.SetDirty(Target);
				}
			}

			private void Inspector() {
				Utility.SetGUIColor(UltiDraw.Grey);
				using(new EditorGUILayout.VerticalScope ("Box")) {
					Utility.ResetGUIColor();

					if(Utility.GUIButton("Animation", UltiDraw.DarkGrey, UltiDraw.White)) {
						Target.Inspect = !Target.Inspect;
					}
					
					if(Target.Inspect) {
						using(new EditorGUILayout.VerticalScope ("Box")) {
							Target.TargetBlending = EditorGUILayout.Slider("Target Blending", Target.TargetBlending, 0f, 1f);
							Target.GaitTransition = EditorGUILayout.Slider("Gait Transition", Target.GaitTransition, 0f, 1f);
							Target.TrajectoryCorrection = EditorGUILayout.Slider("Trajectory Correction", Target.TrajectoryCorrection, 0f, 1f);
						}
					}
					
				}
			}
	}
	#endif
}